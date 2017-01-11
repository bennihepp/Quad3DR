//==================================================
// viewpoint_planner_graph.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 5, 2017
//==================================================

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

template <typename Iterator>
bool ViewpointPlanner::findAndAddShortestMotions(const ViewpointEntryIndex from_index, Iterator to_index_first, Iterator to_index_last) {
  std::cout << "Searching shortest path from new viewpoint " << from_index
      << " to " << (to_index_last - to_index_first) << " other viewpoints" << std::endl;
  for (Iterator it = to_index_first; it != to_index_last; ++it) {
    const ViewpointEntryIndex to_index = it->viewpoint_index;
//    std::cout << "  searching path to viewpoint " << to_index << std::endl;
    bool found = findAndAddShortestMotion(from_index, to_index);
    if (!found) {
      return false;
    }
  }
  return true;
}
