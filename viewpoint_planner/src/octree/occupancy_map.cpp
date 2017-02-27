//==================================================
// occupancy_map.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 14, 2016
//==================================================

#include <src/octree/occupancy_map.h>

template <>
OccupancyMap<OccupancyNode>::StaticMemberInitializer OccupancyMap<OccupancyNode>::ocTreeMemberInit{};

template <>
OccupancyMap<AugmentedOccupancyNode>::StaticMemberInitializer OccupancyMap<AugmentedOccupancyNode>::ocTreeMemberInit{};

template <>
std::string OccupancyMap<OccupancyNode>::getTreeType() const {
  return "Ait_OccupancyMap_OccupancyNode";
}

template <>
std::string OccupancyMap<AugmentedOccupancyNode>::getTreeType() const {
  return "Ait_OccupancyMap_AugmentedOccupancyNode";
}

template <>
std::unique_ptr<OccupancyMap<AugmentedOccupancyNode>> OccupancyMap<AugmentedOccupancyNode>::read(const std::string& filename) {
  std::unique_ptr<OccupancyMap<AugmentedOccupancyNode>> tree(reinterpret_cast<OccupancyMap<AugmentedOccupancyNode>*>(octomap::AbstractOcTree::read(filename)));

  using TreeNavigatorType = OccupancyMap<AugmentedOccupancyNode>::TreeNavigatorType;

  // Update parent pointers
  std::stack<TreeNavigatorType> node_stack;
  node_stack.push(TreeNavigatorType(tree.get(), tree->getRootKey(), tree->getRoot(), 0));
  while (!node_stack.empty()) {
    TreeNavigatorType nav = node_stack.top();
    node_stack.pop();
    if (nav.hasChildren()) {
      for (size_t i = 0; i < 8; ++i) {
        if (nav.hasChild(i)) {
          TreeNavigatorType child_nav = nav.child(i);
          child_nav->setParent(nav.getNode());
          node_stack.push(child_nav);
        }
      }
    }
  }

  return std::move(tree);
}

OccupancyMap<AugmentedOccupancyNode>* convertToAugmentedMap(const OccupancyMap<OccupancyNode>* input_tree) {
  using OutputTreeNavigatorType = OccupancyMap<AugmentedOccupancyNode>::TreeNavigatorType;
  using InputConstTreeNavigatorType = OccupancyMap<OccupancyNode>::ConstTreeNavigatorType;

  OccupancyMap<AugmentedOccupancyNode>* output_tree = new OccupancyMap<AugmentedOccupancyNode>(input_tree->getResolution());
  output_tree->createRoot();

  size_t unknown_count = 0;
  size_t occupied_count = 0;
  size_t free_count = 0;
  size_t leaf_count = 0;

  // Copy input tree into augmented tree and initialize parent pointer
  std::stack<std::pair<InputConstTreeNavigatorType, OutputTreeNavigatorType>> node_stack;
  node_stack.push(std::make_pair(
      InputConstTreeNavigatorType::getRootNavigator(input_tree),
      OutputTreeNavigatorType::getRootNavigator(output_tree)));
  while (!node_stack.empty()) {
    InputConstTreeNavigatorType input_nav = node_stack.top().first;
    OutputTreeNavigatorType output_nav = node_stack.top().second;
    node_stack.pop();
    static_cast<OccupancyNode*>(*output_nav)->copyData(*input_nav.getNode());
    output_nav->setObservationCountSum(input_nav->getObservationCount());
    if (input_nav.hasChildren()) {
      output_tree->allocNodeChildren(*output_nav);
      for (size_t i = 0; i < 8; ++i) {
        if (input_nav.hasChild(i)) {
          InputConstTreeNavigatorType input_child_nav = input_nav.child(i);
          if (input_tree->isNodeOccupied(*input_child_nav) || input_tree->isNodeUnknown(*input_child_nav)) {
            output_tree->allocNodeChild(*output_nav, i);
            OutputTreeNavigatorType output_child_nav = output_nav.child(i);
            output_child_nav->setParent(*output_nav);
            output_child_nav->setWeight(0);
            node_stack.push(std::make_pair(input_child_nav, output_child_nav));
          }
        }
      }
    }
    else {
      ++leaf_count;
      if (output_nav->getObservationCount() == 0) {
        ++unknown_count;
      }
      else if (output_nav->getOccupancy() > 0.55f) {
        ++occupied_count;
      }
      else {
        ++free_count;
      }
    }
  }
  std::cout << "leaf_count=" << leaf_count << ", unknown=" << unknown_count << ", occupied=" << occupied_count << ", free=" << free_count << std::endl;

  // Update observation count sum of inner nodes
  output_tree->updateInnerOccupancy();

  return output_tree;
}
