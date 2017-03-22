//==================================================
// viewpoint_planner_motion.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 9, 2017
//==================================================

#include "viewpoint_planner.h"
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>

#pragma GCC optimize("O0")

bool ViewpointPlanner::hasViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const {
  return viewpoint_graph_motions_.count(ViewpointIndexPair(from_index, to_index)) > 0;
}

ViewpointPlanner::ViewpointMotion ViewpointPlanner::getViewpointMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) const {
  const ViewpointIndexPair vip(from_index, to_index);
#if !AIT_RELEASE
  if (viewpoint_graph_motions_.count(vip) == 0) {
    std::cout << "ERROR: No viewpoint motion from " << from_index << " to " << to_index << std::endl;
  }
#endif
  const ViewpointMotion& motion = viewpoint_graph_motions_.at(vip);
  if (to_index < from_index) {
    ViewpointMotion motion_copy(motion);
    motion_copy.reverse();
    return motion_copy;
  }
  else {
    return motion;
  }
}

void ViewpointPlanner::addViewpointMotion(const ViewpointMotion& motion) {
  ViewpointMotion motion_copy = motion;
  addViewpointMotion(std::move(motion_copy));
}

void ViewpointPlanner::addViewpointMotion(ViewpointMotion&& motion) {
  if (motion.toIndex() < motion.fromIndex()) {
    motion.reverse();
  }
  const ViewpointEntryIndex from_index = motion.fromIndex();
  const ViewpointEntryIndex to_index = motion.toIndex();
#if !AIT_RELEASE
  AIT_ASSERT(from_index < to_index);
#endif
  // Currently graph is unidirectional so edges are automatically symmetric.
  const FloatType distance = motion.distance();
  viewpoint_graph_.addEdgeByNode(from_index, to_index, distance);
  viewpoint_graph_components_valid_ = false;
  const ViewpointIndexPair vip(from_index, to_index);
  const auto it = viewpoint_graph_motions_.find(vip);
  if (it == viewpoint_graph_motions_.end()) {
    viewpoint_graph_motions_.emplace(vip, std::move(motion));
  }
  else {
    it->second = std::move(motion);
  }
#if !AIT_RELEASE
  // Assertions to check consistency between graph edges and motions
  const FloatType motion_distance = getViewpointMotion(from_index, to_index).distance();
  const FloatType motion_weight = viewpoint_graph_.getWeight(from_index, to_index);
  AIT_ASSERT(ait::isApproxEqual(motion_distance, motion_weight, FloatType(1e-2)));
#endif
}

void ViewpointPlanner::computeViewpointMotions() {
  std::cout << "Computing motions on viewpoint graph" << std::endl;
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    const ViewpointEntryIndex from_index = it.node();
    if (from_index < num_real_viewpoints_) {
      continue;
    }
    std::vector<ViewpointPlanner::ViewpointMotion> motions = findViewpointMotions(from_index);
    std::cout << "Found " << motions.size() << " connections from viewpoint " << from_index << std::endl;

    std::unique_lock<std::mutex> lock(mutex_);
    for (ViewpointMotion& motion : motions) {
      addViewpointMotion(std::move(motion));
    }
    lock.unlock();
  }

//  // Print info on connection graph
//  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
//    std::cout << "edges for node " << it.node() << std::endl;
//    auto edges = viewpoint_graph_.getEdgesByNode(it.node());
//    for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) {
//      std::cout << "  edge to " << it2.targetNode() << ", from " << it2.sourceNode() << ", weight " << it2.weight() << std::endl;
//    }
//  }

//  std::cout << "Densifying edges on viewpoint graph" << std::endl;
//  // Densify viewpoint motions
//  std::vector<std::tuple<ViewpointEntryIndex, ViewpointEntryIndex, FloatType>> new_edges;
//  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
//    const ViewpointEntryIndex index = it.node();
//    std::stack<std::tuple<ViewpointEntryIndex, FloatType, std::size_t>> node_stack;
//    std::unordered_set<ViewpointEntryIndex> visited_set;
//    node_stack.push(std::make_tuple(index, 0, 0));
//    while (!node_stack.empty()) {
//      ViewpointEntryIndex stack_index = std::get<0>(node_stack.top());
//      FloatType acc_motion_distance = std::get<1>(node_stack.top());
//      std::size_t depth = std::get<2>(node_stack.top());
//      node_stack.pop();
//      if (depth >= options_.viewpoint_motion_densification_max_depth) {
//        continue;
//      }
//      auto edges = viewpoint_graph_.getEdgesByNode(stack_index);
//      for (auto it = edges.begin(); it != edges.end(); ++it) {
//        ViewpointEntryIndex other_index = it.targetNode();
//        if (visited_set.count(other_index) > 0) {
//          continue;
//        }
//        visited_set.emplace(other_index);
//        FloatType motion_distance = it.weight();
//        node_stack.push(std::make_tuple(other_index, acc_motion_distance + motion_distance, depth + 1));
//        if (depth >= 2) {
//          new_edges.push_back(std::make_tuple(index, other_index, acc_motion_distance + motion_distance));
//        }
//      }
//    }
//  }
//  std::cout << "Adding " << new_edges.size() << " edges to the viewpoint graph" << std::endl;
//  for (const auto& tuple : new_edges) {
//    // TODO: Add symmetric edges?
//    viewpoint_graph_.addEdgeByNode(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
//    viewpoint_graph_.addEdgeByNode(std::get<1>(tuple), std::get<0>(tuple), std::get<2>(tuple));
//  }
//  for (const auto& index : viewpoint_graph_) {
//    std::cout << "Node " << index << " has " << viewpoint_graph_.getEdgesByNode(index).size() << " edges" << std::endl;
//  }
  std::cout << "Done" << std::endl;
}

std::vector<ViewpointPlanner::ViewpointMotion>
ViewpointPlanner::findViewpointMotions(const ViewpointEntryIndex from_index) {
//  ait::Timer timer;
  const ViewpointEntry& from_viewpoint = viewpoint_entries_[from_index];
  // Find motion to other viewpoints in the graph
  const std::size_t dist_knn = options_.viewpoint_motion_max_neighbors;
  const FloatType max_dist_square = options_.viewpoint_motion_max_dist_square;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(from_viewpoint.viewpoint.pose().getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t num_connections = 0;
  std::vector<ViewpointMotion> motions;
#if !AIT_DEBUG
  // We run in multiple threads so make sure that the visible sparse points are cached.
  // Otherwise the OpenGL context and poisson mesh has to be initialized again and again in each thread.
  getCachedVisibleSparsePoints(from_index);
  for (std::size_t i = 0; i < knn_indices.size(); ++i) {
    const ViewpointANN::IndexType to_index = knn_indices[i];
    getCachedVisibleSparsePoints(to_index);
  }
#pragma omp parallel for
#endif
  for (std::size_t i = 0; i < knn_indices.size(); ++i) {
    const ViewpointANN::IndexType to_index = knn_indices[i];
    const ViewpointANN::DistanceType dist_square = knn_distances[i];
//    std::cout << "dist_square=" << dist_square << std::endl;
    if (dist_square >= max_dist_square) {
      continue;
    }
    // Do not consider real point for finding motion paths
    if (to_index < num_real_viewpoints_) {
      continue;
    }
    if (from_index == to_index) {
      continue;
    }
    const bool matchable = isSparseMatchable(from_index, to_index);
    if (!matchable) {
      continue;
    }
    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
    SE3Motion se3_motion;
    bool found_motion;
    std::tie(se3_motion, found_motion) = motion_planner_.findMotion(from_viewpoint.viewpoint.pose(), to_viewpoint.viewpoint.pose());
    // TODO
    if (found_motion) {
#if !AIT_DEBUG
#pragma omp critical
#endif
      {
        ++num_connections;
        // Additional sanity check
//        FloatType motion_cost = motion.cost;
//        if (!ait::isApproxGreaterEqual(motion_cost * motion_cost, dist_square, 1e-2f)) {
//          FloatType dist_square2 = (from_viewpoint.viewpoint.pose().getWorldPosition() - to_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
//          std::cout << "motion_cost=" << motion_cost << std::endl;
//          std::cout << "motion_cost*motion_cost=" << motion_cost * motion_cost << std::endl;
//          std::cout << "dist_square=" << dist_square << std::endl;
//          std::cout << "dist_square2=" << dist_square2 << std::endl;
//          std::cout << "motion_cost * motion_cost - dist_square=" << motion_cost * motion_cost - dist_square << std::endl;
//        }
#if !AIT_RELEASE
        if (ait::isApproxSmaller(se3_motion.cost * se3_motion.cost, dist_square, 1e-2f)) {
          std::cout << "WARNING: se3_motion.cost * se3_motion.cost < dist_square" << std::endl;
          AIT_PRINT_VALUE(se3_motion.cost * se3_motion.cost);
          AIT_PRINT_VALUE(se3_motion.cost);
          AIT_PRINT_VALUE(dist_square);
          AIT_DEBUG_BREAK;
        }
        FloatType motion_distance = 0;
        if (se3_motion.poses.size() >= 2) {
          for (auto it = se3_motion.poses.begin() + 1; it != se3_motion.poses.end(); ++it) {
            motion_distance += (it->getWorldPosition() - (it - 1)->getWorldPosition()).norm();
          }
        }
        if (!ait::isApproxEqual(se3_motion.distance, motion_distance, FloatType(1e-2))) {
          std::cout << "WARNING: se3_motion.distance != * motion_distance" << std::endl;
          AIT_PRINT_VALUE(se3_motion.distance);
          AIT_PRINT_VALUE(motion_distance);
          AIT_DEBUG_BREAK;
        }
#endif
        ViewpointMotion motion({ from_index, to_index }, { se3_motion });
        motions.emplace_back(motion);
      }
    }
  }
//  timer.printTimingMs("ViewpointPlanner::findMotion()");
  return motions;
}

namespace {
struct ViewpointGraphVertexFloatCompare {
  bool operator()(const std::pair<ViewpointPlanner::ViewpointGraph::Vertex, ViewpointPlanner::FloatType> a,
                  const std::pair<ViewpointPlanner::ViewpointGraph::Vertex, ViewpointPlanner::FloatType> b) const {
    return a.second > b.second;
  }
};
}

ViewpointPlanner::ViewpointMotion ViewpointPlanner::findShortestMotionAStar(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) {
  const bool verbose = false;

  ViewpointGraph::Vertex start = viewpoint_graph_.getVertexByNode(from_index);
  ViewpointGraph::Vertex goal = viewpoint_graph_.getVertexByNode(to_index);
  // Currently the code assumes the same indices for the viewpoint graph and the viewpoint entries
#if !AIT_RELEASE
  AIT_ASSERT(start == from_index);
  AIT_ASSERT(goal == to_index);
#endif

  const typename ViewpointPlanner::ViewpointEntry& goal_viewpoint_entry = viewpoint_entries_[goal];
  const auto astar_heuristic = [&] (const ViewpointGraph::Vertex vertex) -> FloatType {
    const typename ViewpointPlanner::ViewpointEntry& viewpoint_entry = viewpoint_entries_[vertex];
    return (viewpoint_entry.viewpoint.pose().getWorldPosition() - goal_viewpoint_entry.viewpoint.pose().getWorldPosition()).norm();
  };

  // Use greater comparison to get a resulting min-heap (boost uses max-heaps by default)
//  using PriorityQueueType = boost::heap::binomial_heap<std::pair<ViewpointGraph::Vertex, FloatType>,
//      boost::heap::mutable_<true>, boost::heap::compare<ViewpointGraphVertexFloatCompare>>;
  using PriorityQueueType = boost::heap::fibonacci_heap<std::pair<ViewpointGraph::Vertex, FloatType>,
      boost::heap::mutable_<true>, boost::heap::compare<ViewpointGraphVertexFloatCompare>>;
  using PriorityQueueHandle = PriorityQueueType::handle_type;
  PriorityQueueType priority_queue;
  std::vector<std::pair<bool, PriorityQueueHandle>> pq_handles(viewpoint_graph_.size(), std::make_pair(false, PriorityQueueHandle()));
  std::vector<FloatType> distances(viewpoint_graph_.size(), std::numeric_limits<FloatType>::max());
  std::vector<ViewpointGraph::Vertex> predecessors(viewpoint_graph_.size());
  distances[start] = FloatType(0);
  predecessors[start] = start;
//  const PriorityQueueHandle start_handle = priority_queue.push(std::make_pair(start, FloatType(0)));
  const PriorityQueueHandle start_handle = priority_queue.push(std::make_pair(start, astar_heuristic(start)));
  pq_handles[start] = std::make_pair(true, start_handle);

  bool found_goal = false;
//  ait::Timer timer;

  while (!priority_queue.empty()) {
    ViewpointGraph::Vertex current_vertex;
    std::tie(current_vertex, std::ignore) = priority_queue.top();
    const FloatType current_distance = distances[current_vertex];
    priority_queue.pop();
    if (current_vertex == goal) {
      found_goal = true;
      break;
    }
    // TODO: getEdges() is non-const. This should be changed so that this method can be const.
    const ViewpointGraph::OutEdgesWrapper out_edges = viewpoint_graph_.getEdges(current_vertex);
    for (ViewpointGraph::OutEdgeIteratorWrapper it = out_edges.begin(); it != out_edges.end(); ++it) {
      const ViewpointGraph::Vertex new_vertex = it.target();
      const FloatType weight = it.weight();
      const FloatType new_distance = current_distance + weight + options_.viewpoint_motion_penalty_per_graph_vertex;
      if (new_distance < distances[new_vertex]) {
        distances[new_vertex] = new_distance;
        const FloatType new_heuristic_distance = new_distance + astar_heuristic(new_vertex);
        predecessors[new_vertex] = current_vertex;
        if (pq_handles[new_vertex].first) {
          const PriorityQueueHandle handle = pq_handles[new_vertex].second;
          // TODO: Figure out whether to decrease or increase the value
//          priority_queue.update(handle, std::make_pair(new_vertex, new_distance));
          priority_queue.update(handle, std::make_pair(new_vertex, new_heuristic_distance));
//          priority_queue.increase(handle, std::make_pair(new_vertex, new_total_heuristic_distance));
        }
        else {
//          const PriorityQueueHandle handle = priority_queue.push(std::make_pair(new_vertex, new_distance));
          const PriorityQueueHandle handle = priority_queue.push(std::make_pair(new_vertex, new_heuristic_distance));
          pq_handles[new_vertex] = std::make_pair(true, handle);
        }
      }
    }
  }

  if (found_goal) {
//    timer.printTimingMs("Astar search");
    FloatType motion_distance = distances[goal];
    if (verbose) {
      std::cout << "  Found path to " << to_index << " with distance " << motion_distance
          << ", line of sight distance is "
          << (viewpoint_entries_[from_index].viewpoint.pose().getWorldPosition() - viewpoint_entries_[to_index].viewpoint.pose().getWorldPosition()).norm() << std::endl;
    }
    std::cout << "Predecessors:" << std::endl;
    std::vector<ViewpointEntryIndex> reverse_viewpoint_indices;
    for (ViewpointGraph::Vertex v = goal;; v = predecessors[v]) {
      if (verbose) {
        std::cout << "  vertex = " << v << std::endl;
      }
      reverse_viewpoint_indices.push_back(viewpoint_graph_.getNode(v));
      if (predecessors[v] == v) {
        break;
      }
    }
#if !AIT_RELEASE
    AIT_ASSERT(reverse_viewpoint_indices.back() == from_index);
    AIT_ASSERT(reverse_viewpoint_indices.front() == to_index);
#endif
    std::vector<ViewpointEntryIndex> viewpoint_indices;
    viewpoint_indices.reserve(reverse_viewpoint_indices.size());
    std::copy(reverse_viewpoint_indices.rbegin(), reverse_viewpoint_indices.rend(), std::back_inserter(viewpoint_indices));
    ViewpointMotion motion;
    for (auto it = viewpoint_indices.begin() + 1; it != viewpoint_indices.end(); ++it) {
      const ViewpointMotion sub_motion = getViewpointMotion(*(it - 1), *it);
      motion.append(sub_motion);
    }

//    // Densify viewpoint graph motions (add all shortest paths that were found to the graph)
//    for (auto from_it = viewpoint_indices.begin(); from_it != viewpoint_indices.end(); ++from_it) {
//      for (auto to_it = from_it + 1; to_it != viewpoint_indices.end(); ++to_it) {
//        auto last_it = to_it + 1;
//        std::vector<ViewpointEntryIndex> sub_viewpoint_indices;
//        sub_viewpoint_indices.push_back(*from_it);
//        std::vector<SE3Motion> se3_motions;
//        for (auto sub_it = from_it + 1; sub_it != last_it; ++sub_it) {
//          const ViewpointMotion sub_motion = getViewpointMotion(*(sub_it - 1), *sub_it);
//          if (sub_motion.viewpointIndices().size() > 2) {
//            for (auto sub_viewpoint_it = sub_motion.viewpointIndices().begin() + 1;
//                sub_viewpoint_it != sub_motion.viewpointIndices().end() - 1;
//                ++sub_viewpoint_it) {
//              sub_viewpoint_indices.push_back(*sub_viewpoint_it);
//            }
//          }
//          sub_viewpoint_indices.push_back(*sub_it);
//          const std::vector<SE3Motion>& sub_se3_motions = sub_motion.se3Motions();
//          std::copy(sub_se3_motions.begin(), sub_se3_motions.end(), std::back_inserter(se3_motions));
//        }
//        ViewpointMotion motion(sub_viewpoint_indices, se3_motions);
//        addViewpointMotion(std::move(motion));
//      }
//    }

#if !AIT_RELEASE
    AIT_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());
#endif

    return motion;
  }
  else {
    if (verbose) {
      std::cout << "Could not find a motion from " << from_index << " to " << to_index << std::endl;
    }
    return ViewpointMotion();
  }
}

ViewpointPlanner::ViewpointMotion ViewpointPlanner::optimizeViewpointMotion(const ViewpointMotion& motion) const {
  // Note: Assuming random access iterators for ViewpointMotion members
  std::vector<ViewpointEntryIndex> viewpoint_indices;
  std::vector<SE3Motion> se3_motions;
  viewpoint_indices.push_back(motion.viewpointIndices().front());
  auto it = motion.viewpointIndices().begin();
  while (it != motion.viewpointIndices().end() - 1) {
    for (auto other_it = motion.viewpointIndices().end() - 1; other_it > it; --other_it) {
      if (it + 1 == other_it
          || isSparseMatchable(*it, *other_it)) {
        viewpoint_indices.push_back(*other_it);
        bool found_se3_motion = false;
        SE3Motion se3_motion;
        for (auto loop_it = it; loop_it < other_it; ++loop_it) {
          se3_motion.append(motion.se3Motions()[loop_it - motion.viewpointIndices().begin()]);
        }
        if (it + 1 < other_it) {
          SE3Motion new_se3_motion;
          std::tie(new_se3_motion, found_se3_motion) = motion_planner_.findMotion(
              viewpoint_entries_[*it].viewpoint.pose(), viewpoint_entries_[*other_it].viewpoint.pose());
          if (found_se3_motion && new_se3_motion.distance < se3_motion.distance) {
            se3_motion = new_se3_motion;
          }
        }
        se3_motions.push_back(se3_motion);
        it = other_it;
        break;
      }
    }
  }
  ViewpointMotion optimized_motion(viewpoint_indices, se3_motions);

  // TODO
  //  const auto get_motion_pose_lambda = [](const ViewpointMotion& motion, const FloatType fraction_done) -> Pose {
  ////    const Pose& first_pose = motion.poses.front();
  ////    const Pose& last_pose = motion.poses.back();
  //    AIT_ASSERT(fraction_done <= 1);
  //    FloatType accumulated_fraction = 0;
  //    for (auto next_it = motion.poses.begin() + 1; next_it != motion.poses.end(); ++next_it) {
  //      auto prev_it = next_it - 1;
  //      FloatType segment_distance = (next_it->translation() - prev_it->translation()).norm();
  //      FloatType segment_fraction = segment_distance / motion.distance;
  //      if (accumulated_fraction + segment_fraction >= fraction_done) {
  //        const FloatType segment_factor = (fraction_done - accumulated_fraction) / segment_fraction;
  //        AIT_ASSERT(segment_factor <= 1);
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
  //        AIT_PRINT_VALUE(matching_score);
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
  //          AIT_PRINT_VALUE(matching_score);
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
  //  AIT_ASSERT(viewpoint_path->order.size() == viewpoint_path->entries.size());

  return optimized_motion;
}

// AStar
bool ViewpointPlanner::findAndAddShortestMotion(const ViewpointEntryIndex from_index, const ViewpointEntryIndex to_index) {
  const bool verbose = true;

  const ViewpointMotion motion = findShortestMotionAStar(from_index, to_index);
  if (motion.isValid()) {
    const ViewpointMotion optimized_motion = optimizeViewpointMotion(motion);
    if (verbose) {
      std::cout << "Optimized motion from " << motion.viewpointIndices().size() << " viewpoints to "
          << optimized_motion.viewpointIndices().size() << " viewpoints" << std::endl;
    }
    addViewpointMotion(std::move(optimized_motion));
#if !AIT_RELEASE
    AIT_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());
#endif
    return true;
  }
  else {
    if (verbose) {
      std::cout << "Could not find a motion from " << from_index << " to " << to_index << std::endl;
    }
    return false;
  }
}
