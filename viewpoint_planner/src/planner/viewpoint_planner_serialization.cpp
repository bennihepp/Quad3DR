//==================================================
// viewpoint_planner_serialization.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 6, 2017
//==================================================

#include "viewpoint_planner.h"
#include "viewpoint_planner_serialization.h"

void ViewpointPlanner::saveViewpointGraph(const std::string& filename) const {
  std::cout << "Writing viewpoint graph to " << filename << std::endl;
  std::cout << "Graph has " << viewpoint_graph_.numVertices() << " viewpoints"
      << " and " << viewpoint_graph_.numEdges() << " motions" << std::endl;
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  ViewpointEntrySaver ves(viewpoint_entries_, data_->occupied_bvh_);
  oa << num_real_viewpoints_;
  oa << ves;
  oa << viewpoint_graph_;
  oa << viewpoint_graph_motions_;
  std::cout << "Done" << std::endl;
}

void ViewpointPlanner::loadViewpointGraph(const std::string& filename) {
  reset();
  std::cout << "Loading viewpoint graph from " << filename << std::endl;
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  std::size_t new_num_real_viewpoints;
  ia >> new_num_real_viewpoints;
  BH_ASSERT(new_num_real_viewpoints == num_real_viewpoints_);
  ViewpointEntryLoader vel(&viewpoint_entries_, &data_->occupied_bvh_, &virtual_camera_);
  ia >> vel;
  viewpoint_graph_.clear();
  ia >> viewpoint_graph_;
  viewpoint_graph_components_valid_ = false;
  std::cout << "Regenerating approximate nearest neighbor index" << std::endl;
  viewpoint_ann_.clear();
  for (const ViewpointEntry& viewpoint_entry : viewpoint_entries_) {
    viewpoint_ann_.addPoint(viewpoint_entry.viewpoint.pose().getWorldPosition());
  }
  std::cout << "Regenerating density field" << std::endl;
  viewpoint_count_grid_.setAllValues(0);
  for (const ViewpointEntry& viewpoint_entry : viewpoint_entries_) {
    const Vector3& viewpoint_position = viewpoint_entry.viewpoint.pose().getWorldPosition();
    if (viewpoint_count_grid_.isInsideGrid(viewpoint_position)) {
      viewpoint_count_grid_(viewpoint_position) += 1;
    }
    else {
      std::cout << "WARNING: Loaded viewpoint outside of density grid" << std::endl;
    }
  }
  std::cout << "Loading motions" << std::endl;
  ia >> viewpoint_graph_motions_;
  std::cout << "Loaded viewpoint graph with " << viewpoint_entries_.size() << " viewpoints "
      << " and " << viewpoint_graph_.numEdges() << " motions" << std::endl;
  BH_ASSERT(viewpoint_entries_.size() == viewpoint_graph_.numVertices());
  BH_ASSERT(viewpoint_graph_.numEdges() == viewpoint_graph_motions_.size());

  // Consistency check that viewpoint motion distances and graph edge weights are equal
  for (ViewpointEntryIndex viewpoint_index = 0; viewpoint_index < viewpoint_entries_.size(); ++viewpoint_index) {
    const auto edges = viewpoint_graph_.getEdges(viewpoint_index);
    for (auto it = edges.begin(); it != edges.end(); ++it) {
      const FloatType weight = it.weight();
      const FloatType dist = getViewpointMotion(it.sourceNode(), it.targetNode()).distance();
      if (!bh::isApproxEqual(weight, dist, FloatType(1e-2))) {
        std::cout << "it.sourceNode()=" << it.sourceNode() << ", it.targetNode()=" << it.targetNode()
            << ", weight=" << weight << ", dist=" << dist << std::endl;
      }
      BH_ASSERT(it.source() == viewpoint_index);
      BH_ASSERT(it.sourceNode() == viewpoint_index);
      BH_ASSERT(it.target() == it.targetNode());
    }
  }

  std::cout << "Clearing observed voxels for previous camera viewpoints" << std::endl;
  for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
    ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
    viewpoint_entry.voxel_set.clear();
  }
}

void ViewpointPlanner::saveViewpointPath(const std::string& filename) const {
  std::cout << "Writing viewpoint paths to " << filename << std::endl;
  std::cout << "There are " << viewpoint_paths_.size() << " paths."
      << " The best path has " << getBestViewpointPath().entries.size() << " viewpoints" << std::endl;
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  ViewpointPathSaver vps(viewpoint_paths_, viewpoint_paths_data_, data_->occupied_bvh_);
  oa << vps;

  // Save sparse matching viewpoints
  std::unordered_map<ViewpointEntryIndex, Pose> sparse_matching_viewpoint_poses;
  for (const ViewpointPath& viewpoint_path : viewpoint_paths_) {
    for (auto from_it = viewpoint_path.order.begin(); from_it != viewpoint_path.order.end(); ++from_it) {
      auto to_it = from_it + 1;
      if (to_it == viewpoint_path.order.end()) {
        to_it = viewpoint_path.order.begin();
      }
      const ViewpointEntryIndex from_index = viewpoint_path.entries[*from_it].viewpoint_index;
      const ViewpointEntryIndex to_index = viewpoint_path.entries[*to_it].viewpoint_index;
      if (hasViewpointMotion(from_index, to_index)) {
        const ViewpointMotion &motion = getViewpointMotion(from_index, to_index);
        for (auto vi_it = motion.viewpointIndices().begin() + 1;
             vi_it != motion.viewpointIndices().end() - 1; ++vi_it) {
          const Pose &pose = viewpoint_entries_[*vi_it].viewpoint.pose();
          sparse_matching_viewpoint_poses.emplace(*vi_it, pose);
        }
      }
    }
  }
  oa << sparse_matching_viewpoint_poses;

  // Save viewpoint motion of path
  std::unordered_map<ViewpointIndexPair, ViewpointMotion, ViewpointIndexPair::Hash> local_viewpoint_path_motions;
  for (const ViewpointPath& viewpoint_path : viewpoint_paths_) {
    for (auto it1 = viewpoint_path.entries.begin(); it1 != viewpoint_path.entries.end(); ++it1) {
      for (auto it2 = viewpoint_path.entries.begin(); it2 != viewpoint_path.entries.end(); ++it2) {
        if (it1 != it2) {
          const ViewpointIndexPair vip(it1->viewpoint_index, it2->viewpoint_index);
          const auto it = viewpoint_graph_motions_.find(vip);
          if (it != viewpoint_graph_motions_.end()) {
            const ViewpointMotion& motion = it->second;
            local_viewpoint_path_motions.emplace(vip, motion);
          }
//          else {
//            const ViewpointMotion motion = findShortestMotionAStar(it1->viewpoint_index, it2->viewpoint_index);
//            if (motion.isValid()) {
//              const ViewpointMotion optimized_motion = optimizeViewpointMotion(motion);
//              local_viewpoint_path_motions.emplace(vip, optimized_motion);
//            }
//            else {
//              std::cout << "WARNING: No motion from path viewpoint " <<
//                  it1->viewpoint_index << " to path viewpoint " << it2->viewpoint_index << std::endl;
//            }
//          }
        }
      }
    }
  }
  oa << local_viewpoint_path_motions;
  std::cout << "Done" << std::endl;
  reportViewpointPathsStats();

  for (std::size_t j = 0; j < viewpoint_paths_.size(); ++j) {
    const ViewpointPath& viewpoint_path = viewpoint_paths_[j];
    std::cout << "Viewpoint path " << j << std::endl;
    if (!viewpoint_path.order.empty()) {
      std::cout << "Path order" << std::endl;
      for (std::size_t i = 0; i < viewpoint_paths_[0].entries.size(); ++i) {
        std::cout << "  " << i << ": " << viewpoint_paths_[0].entries[i].viewpoint_index << ", order=" << viewpoint_paths_[0].order[i] << std::endl;
      }
    }
    std::cout << "Local motions" << std::endl;
    for (const auto& motion_entry : local_viewpoint_path_motions) {
      const ViewpointIndexPair& vip = motion_entry.first;
      std::cout << "  " << vip.index1 << " <-> " << vip.index2 << std::endl;
    }
  }
}

#pragma GCC optimize("O0")
void ViewpointPlanner::loadViewpointPath(const std::string& filename) {
  resetViewpointPaths();
  std::unique_lock<std::mutex> lock(mutex_);
  std::cout << "Loading viewpoint paths from " << filename << std::endl;
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  std::vector<ViewpointPath> tmp_viewpoint_paths;
  std::vector<ViewpointPathComputationData> tmp_viewpoint_paths_data;
  ViewpointPathLoader vpl(&tmp_viewpoint_paths, &tmp_viewpoint_paths_data,
                          &data_->occupied_bvh_, viewpoint_entries_, &virtual_camera_);
  ia >> vpl;
  // Match old entries in viewpoint path to current entries in viewpoint graph
  std::unordered_map<ViewpointEntryIndex, ViewpointEntryIndex> old_to_new_viewpoint_mapping;
  for (ViewpointPath& viewpoint_path : tmp_viewpoint_paths) {
    for (std::size_t i = 0; i < viewpoint_path.entries.size(); ++i) {
      ViewpointPathEntry& entry = viewpoint_path.entries[i];
      bool found;
      ViewpointEntryIndex matching_viewpoint_index;
      std::tie(found, matching_viewpoint_index) = findViewpointEntryWithPose(entry.viewpoint.pose());
      if (!found) {
        // Compute observed voxels and information and add to viewpoint graph
          std::pair<VoxelWithInformationSet, FloatType> raycast_result =
              getRaycastHitVoxelsWithInformationScore(entry.viewpoint);
          VoxelWithInformationSet& voxel_set = raycast_result.first;
          FloatType total_information = raycast_result.second;
          const ViewpointEntryIndex new_viewpoint_index = addViewpointEntryWithoutLock(
              ViewpointEntry(entry.viewpoint, total_information, std::move(voxel_set)));
          matching_viewpoint_index = new_viewpoint_index;
          // Try to find motion to existing viewpoints
          bool found_motion = false;
          SE3Motion se3_motion;
          if (viewpoint_entries_.size() > num_real_viewpoints_) {
            const Pose& pose = entry.viewpoint.pose();
            if (isValidObjectPosition(pose.getWorldPosition(), drone_bbox_)) {
              const std::size_t knn = options_.viewpoint_motion_max_neighbors;
              static std::vector<ViewpointANN::IndexType> knn_indices;
              static std::vector<ViewpointANN::DistanceType> knn_distances;
              knn_indices.resize(knn);
              knn_distances.resize(knn);
              viewpoint_ann_.knnSearch(entry.viewpoint.pose().getWorldPosition(), knn, &knn_indices, &knn_distances);
              for (std::size_t j = 0; j < knn_distances.size(); ++j) {
                const ViewpointANN::DistanceType dist_square = knn_distances[j];
                const ViewpointEntryIndex other_viewpoint_index = knn_indices[j];
                if (other_viewpoint_index == matching_viewpoint_index || other_viewpoint_index < num_real_viewpoints_) {
                  continue;
                }
                const Pose& other_pose = viewpoint_entries_[other_viewpoint_index].viewpoint.pose();
                if (!isValidObjectPosition(other_pose.getWorldPosition(), drone_bbox_)) {
                  std::cout << "WARNING: Nearest neighbor of viewpoint path is not a valid object position" << std::endl;
                  continue;
                }
                const bool matchable = isSparseMatchable2(new_viewpoint_index, other_viewpoint_index);
                if (!matchable) {
                  continue;
                }
                std::tie(se3_motion, found_motion) = motion_planner_.findMotion(pose, other_pose);
                if (found_motion) {
                  ViewpointMotion viewpoint_motion({ new_viewpoint_index, other_viewpoint_index }, { se3_motion });
                  addViewpointMotion(std::move(viewpoint_motion));
                  break;
                }
              }
              if (!found_motion) {
                std::cout << "WARNING: Unable to find motion for loaded viewpoint entry to existing viewpoints" << std::endl;
                std::cout << i << " " << viewpoint_path.entries[i].viewpoint_index << " " << matching_viewpoint_index << std::endl;
                if (viewpoint_path.order.empty()) {
                  std::cout << "No order" << std::endl;
                }
                else {
                  std::cout << viewpoint_path.order[i] << std::endl;
                }
              }
            }
            else {
              std::cout << "WARNING: Entry in viewpoint path is not a valid object position" << std::endl;
            }
          }
      }
      old_to_new_viewpoint_mapping.emplace(entry.viewpoint_index, matching_viewpoint_index);
      entry.viewpoint_index = matching_viewpoint_index;
    }
  }
  std::cout << "Viewpoint index mapping:" << std::endl;
  for (const auto& entry : old_to_new_viewpoint_mapping) {
    std::cout << "  " << entry.first << " <-> " << entry.second << std::endl;
  }

  // Load sparse matching viewpoints
  std::unordered_map<ViewpointEntryIndex, Pose> sparse_matching_viewpoint_poses;
  ia >> sparse_matching_viewpoint_poses;
  for (auto& entry : sparse_matching_viewpoint_poses) {
    const ViewpointEntryIndex old_index = entry.first;
    const Pose& pose = entry.second;
    bool found;
    ViewpointEntryIndex matching_viewpoint_index;
    std::tie(found, matching_viewpoint_index) = findViewpointEntryWithPose(pose);
    if (!found) {
      const VoxelWithInformationSet voxel_set;
      const FloatType total_information = 0;
      const ViewpointEntryIndex new_viewpoint_index = addViewpointEntryWithoutLock(
              ViewpointEntry(getVirtualViewpoint(pose), total_information, voxel_set));
      matching_viewpoint_index = new_viewpoint_index;
    }
    old_to_new_viewpoint_mapping.emplace(old_index, matching_viewpoint_index);
  }

  // Load viewpoint motions of path
  std::unordered_map<ViewpointIndexPair, ViewpointMotion, ViewpointIndexPair::Hash> local_viewpoint_path_motions;
  ia >> local_viewpoint_path_motions;
  // TODO: Compute hash of viewpoint graph and save with path. Then only load motions if hashes are equal.
  for (const auto& motion_entry : local_viewpoint_path_motions) {
//    const ViewpointIndexPair& vip = motion_entry.first;
    const ViewpointMotion& motion = motion_entry.second;
    std::vector<ViewpointEntryIndex> new_motion_indices;
    std::vector<SE3Motion> new_se3_motions;
    bool new_motion_valid = true;
    for (ViewpointMotion::ConstIterator it = motion.begin(); it != motion.end(); ++it) {
      const ViewpointEntryIndex old_index = *it;
      const auto mapping_it = old_to_new_viewpoint_mapping.find(old_index);
      if (mapping_it == old_to_new_viewpoint_mapping.end()) {
        new_motion_valid = false;
        break;
      }
      const ViewpointEntryIndex new_index = mapping_it->second;
      new_motion_indices.push_back(new_index);
      std::cout << old_index << " " << new_index << std::endl;
      BH_ASSERT(new_index < viewpoint_graph_.numVertices());
    }
    if (new_motion_valid) {
      ViewpointMotion new_motion(new_motion_indices, motion.se3Motions());
      addViewpointMotion(std::move(new_motion));
    }
  }
  std::cout << "Recomputing observed voxel sets and sorted information lists" << std::endl;
//  std::cout << "Recomputing sorted information lists" << std::endl;
  viewpoint_paths_initialized_ = true;
  viewpoint_paths_.clear();
  viewpoint_paths_data_.clear();
  viewpoint_paths_.resize(tmp_viewpoint_paths.size());
  viewpoint_paths_data_.resize(tmp_viewpoint_paths_data.size());
#pragma omp parallel for
  for (std::size_t i = 0; i < tmp_viewpoint_paths.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    const ViewpointPath& tmp_viewpoint_path = tmp_viewpoint_paths[i];
    initializeViewpointPathInformations(&viewpoint_path, &comp_data);
    updateViewpointPathInformations(&viewpoint_path, &comp_data);
    for (const ViewpointPathEntry& path_entry : tmp_viewpoint_path.entries) {
      ViewpointPathEntry new_path_entry = path_entry;
      addViewpointPathEntryWithoutLock(&viewpoint_path, &comp_data, new_path_entry);
    }
    viewpoint_path.order = tmp_viewpoint_path.order;
  }
  std::cout << "Ensuring connectivity of viewpoints on paths" << std::endl;
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    comp_data.num_connected_entries = 0;
//    ensureConnectedViewpointPath(&viewpoint_path, &comp_data);
    for (auto from_it = viewpoint_path.order.begin(); from_it != viewpoint_path.order.end(); ++from_it) {
      for (auto to_it = viewpoint_path.order.begin(); to_it != from_it; ++to_it) {
        const ViewpointEntryIndex from_index = viewpoint_path.entries[*from_it].viewpoint_index;
        const ViewpointEntryIndex to_index = viewpoint_path.entries[*to_it].viewpoint_index;
        if (!hasViewpointMotion(from_index, to_index)) {
          if (!isValidObjectPosition(viewpoint_entries_[from_index].viewpoint.pose().getWorldPosition(), drone_bbox_)) {
            std::cout << "WARNING: Viewpoint " << from_index << " is not a valid position" << std::endl;
            continue;
          }
          if (!isValidObjectPosition(viewpoint_entries_[to_index].viewpoint.pose().getWorldPosition(), drone_bbox_)) {
            std::cout << "WARNING: Viewpoint " << from_index << " is not a valid position" << std::endl;
            continue;
          }
          std::cout << "Finding motion from " << viewpoint_path.entries[*from_it].viewpoint_index << " to "
                    << viewpoint_path.entries[*to_it].viewpoint_index << std::endl;
          const bool found_motion = findAndAddShortestMotion(
                  from_index, to_index);
          if (!found_motion) {
            std::cout << "WARNING: Could not find motion path from viewpoint " << *from_it << " to " << *to_it
                      << std::endl;
          }
        }
      }
      ++comp_data.num_connected_entries;
    }
  }
  reportViewpointPathsStats();
  std::cout << "Loaded " << viewpoint_paths_.size() << " viewpoint paths."
      << " The best path has " << getBestViewpointPath().entries.size() << " viewpoints" << std::endl;

  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
//    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    std::cout << "order for path " << i << std::endl;
    for (std::size_t i : viewpoint_path.order) {
      const ViewpointPathEntry& path_entry = viewpoint_path.entries[i];
      std::cout << "path_index=" << i << ", viewpoint_index=" << path_entry.viewpoint_index << std::endl;
    }
  }
  // Recompute connected components
  getConnectedComponents();
}
