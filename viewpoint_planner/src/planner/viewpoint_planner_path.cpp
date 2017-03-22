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
#include <boost/graph/metric_tsp_approx.hpp>
#include <bh/algorithm.h>

using std::swap;

void ViewpointPlanner::addViewpointPathEntry(const size_t viewpoint_path_index,
                                             const ViewpointEntryIndex viewpoint_index,
                                             const bool ignore_observed_voxels /*= false*/) {
  if (!viewpoint_paths_initialized_) {
    initializeViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
  }
  std::unique_lock<std::mutex> lock = acquireLock();

  ViewpointPathEntry path_entry;
  path_entry.viewpoint_index = viewpoint_index;
  path_entry.viewpoint = viewpoint_entries_[viewpoint_index].viewpoint;
  path_entry.mvs_viewpoint = true;
  ViewpointPath* viewpoint_path = &viewpoint_paths_[viewpoint_path_index];
  ViewpointPathComputationData* comp_data = &viewpoint_paths_data_[viewpoint_path_index];
  addViewpointPathEntryWithoutLock(
          viewpoint_path,
          comp_data,
          path_entry,
          ignore_observed_voxels);
}

bool ViewpointPlanner::addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                                           const ViewpointEntryIndex viewpoint_index,
                                                           const bool ignore_observed_voxels /*= false*/) {
  const bool verbose = true;

  if (!viewpoint_paths_initialized_) {
    initializeViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
  }
  std::unique_lock<std::mutex> lock = acquireLock();

  ViewpointPath& viewpoint_path = viewpoint_paths_[viewpoint_path_index];
  ViewpointPathComputationData& comp_data = viewpoint_paths_data_[viewpoint_path_index];

  ViewpointPathEntry path_entry;
  path_entry.viewpoint_index = viewpoint_index;
  path_entry.viewpoint = viewpoint_entries_[viewpoint_index].viewpoint;
  path_entry.mvs_viewpoint = true;

  ViewpointEntryIndex stereo_viewpoint_index = (ViewpointEntryIndex)-1;
  bool stereo_viewpoint_already_in_path = false;
  const bool ignore_sparse_matching = false;
  const bool ignore_graph_component = false;
  // Try to find a matching stereo pair. Otherwise return failure.
  std::tie(stereo_viewpoint_already_in_path, stereo_viewpoint_index) = findMatchingStereoViewpointWithoutLock(
          viewpoint_path, comp_data, path_entry.viewpoint_index,
          ignore_sparse_matching, ignore_graph_component);
  if (!stereo_viewpoint_already_in_path && stereo_viewpoint_index == (ViewpointEntryIndex)-1) {
    if (verbose) {
      std::cout << "Could not find any usable stereo viewpoint for viewpoint " << path_entry.viewpoint_index << std::endl;
    }
    return false;
  }

  if (!stereo_viewpoint_already_in_path) {
    ViewpointPathEntry stereo_path_entry;
    stereo_path_entry.viewpoint_index = stereo_viewpoint_index;
    stereo_path_entry.viewpoint = viewpoint_entries_[stereo_viewpoint_index].viewpoint;
    stereo_path_entry.mvs_viewpoint = true;
    addViewpointPathEntryWithoutLock(
            &viewpoint_paths_[viewpoint_path_index],
            &viewpoint_paths_data_[viewpoint_path_index],
            stereo_path_entry,
            ignore_observed_voxels);
  }

  addViewpointPathEntryWithoutLock(
          &viewpoint_path,
          &comp_data,
          path_entry,
          ignore_observed_voxels);

  return true;
}

void ViewpointPlanner::addViewpointPathEntry(const size_t viewpoint_path_index,
                                             const Pose& pose,
                                             const bool ignore_observed_voxels /*= false*/) {
  if (!viewpoint_paths_initialized_) {
    initializeViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
  }
  std::unique_lock<std::mutex> lock = acquireLock();

  const Viewpoint viewpoint = getVirtualViewpoint(pose);
  const bool ignore_voxels_with_zero_information = true;
  std::pair<VoxelWithInformationSet, FloatType> raycast_result =
          getRaycastHitVoxelsWithInformationScore(viewpoint, ignore_voxels_with_zero_information);
  VoxelWithInformationSet& voxel_set = raycast_result.first;
  const FloatType total_information = raycast_result.second;
  const ViewpointEntryIndex new_viewpoint_index = addViewpointEntryWithoutLock(
          ViewpointEntry(viewpoint, total_information, std::move(voxel_set)));

  ViewpointPathEntry path_entry;
  path_entry.viewpoint_index = new_viewpoint_index;
  path_entry.viewpoint = viewpoint;
  path_entry.mvs_viewpoint = true;
  ViewpointPath* viewpoint_path = &viewpoint_paths_[viewpoint_path_index];
  ViewpointPathComputationData* comp_data = &viewpoint_paths_data_[viewpoint_path_index];
  addViewpointPathEntryWithoutLock(
          viewpoint_path,
          comp_data,
          path_entry,
          ignore_observed_voxels);
}

bool ViewpointPlanner::addViewpointPathEntryWithStereoPair(const size_t viewpoint_path_index,
                                                           const Pose& pose,
                                                           const bool ignore_observed_voxels /*= false*/) {
  const bool verbose = true;
  if (!viewpoint_paths_initialized_) {
    initializeViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
  }
  std::unique_lock<std::mutex> lock = acquireLock();

  ViewpointPath& viewpoint_path = viewpoint_paths_[viewpoint_path_index];
  ViewpointPathComputationData& comp_data = viewpoint_paths_data_[viewpoint_path_index];

  const Viewpoint viewpoint = getVirtualViewpoint(pose);
  const bool ignore_voxels_with_zero_information = true;
  std::pair<VoxelWithInformationSet, FloatType> raycast_result =
          getRaycastHitVoxelsWithInformationScore(viewpoint, ignore_voxels_with_zero_information);
  VoxelWithInformationSet& voxel_set = raycast_result.first;
  const FloatType total_information = raycast_result.second;
  const ViewpointEntryIndex new_viewpoint_index = addViewpointEntryWithoutLock(
          ViewpointEntry(viewpoint, total_information, std::move(voxel_set)));

  ViewpointPathEntry path_entry;
  path_entry.viewpoint_index = new_viewpoint_index;
  path_entry.viewpoint = viewpoint;
  path_entry.mvs_viewpoint = true;

  ViewpointEntryIndex stereo_viewpoint_index = (ViewpointEntryIndex)-1;
  bool stereo_viewpoint_already_in_path = false;
  const bool ignore_sparse_matching = true;
  const bool ignore_graph_component = true;
    // Try to find a matching stereo pair. Otherwise return failure.
  std::tie(stereo_viewpoint_already_in_path, stereo_viewpoint_index) = findMatchingStereoViewpointWithoutLock(
          viewpoint_path, comp_data, path_entry.viewpoint_index,
          ignore_sparse_matching, ignore_graph_component);
  if (!stereo_viewpoint_already_in_path && stereo_viewpoint_index == (ViewpointEntryIndex)-1) {
    if (verbose) {
      std::cout << "Could not find any usable stereo viewpoint for viewpoint " << path_entry.viewpoint_index << std::endl;
    }
    return false;
  }

  if (!stereo_viewpoint_already_in_path) {
    ViewpointPathEntry stereo_path_entry;
    stereo_path_entry.viewpoint_index = stereo_viewpoint_index;
    stereo_path_entry.viewpoint = viewpoint_entries_[stereo_viewpoint_index].viewpoint;
    stereo_path_entry.mvs_viewpoint = true;
    addViewpointPathEntryWithoutLock(
            &viewpoint_paths_[viewpoint_path_index],
            &viewpoint_paths_data_[viewpoint_path_index],
            stereo_path_entry,
            ignore_observed_voxels);
  }

  addViewpointPathEntryWithoutLock(
          &viewpoint_path,
          &comp_data,
          path_entry,
          ignore_observed_voxels);

  return true;
}

void ViewpointPlanner::addViewpointPathEntryWithoutLock(ViewpointPath* viewpoint_path,
                                                        ViewpointPathComputationData* comp_data,
                                                        const ViewpointPathEntry& new_path_entry,
                                                        const bool ignore_observed_voxels /*= false*/) {
  const ViewpointEntry& viewpoint_entry = viewpoint_entries_[new_path_entry.viewpoint_index];
  FloatType new_viewpoint_information = viewpoint_entry.total_information;
  if (!ignore_observed_voxels) {
    new_viewpoint_information = 0;
    for (const VoxelWithInformation& vi : viewpoint_entry.voxel_set) {
      const FloatType observation_information = options_.viewpoint_information_factor * vi.information;
      FloatType novel_observation_information;
      auto it = viewpoint_path->observed_voxel_map.find(vi.voxel);
      if (it == viewpoint_path->observed_voxel_map.end()) {
        viewpoint_path->observed_voxel_map.emplace(vi.voxel, observation_information);
        novel_observation_information = observation_information;
      }
      else {
        const WeightType voxel_weight = vi.voxel->getObject()->weight;
        novel_observation_information = std::min(observation_information, voxel_weight - it->second);
        AIT_ASSERT(it->second <= voxel_weight);
        if (it->second <= voxel_weight) {
          it->second += observation_information;
          if (it->second > voxel_weight) {
            it->second = voxel_weight;
          }
        }
      }
#if !AIT_RELEASE
      AIT_ASSERT(novel_observation_information >= 0);
#endif
      new_viewpoint_information += novel_observation_information;
    }
  }
//  viewpoint_path->observed_voxel_set.insert(viewpoint_entry.voxel_set.cbegin(), viewpoint_entry.voxel_set.cend());
  ViewpointPathEntry new_path_entry_copy = new_path_entry;
  new_path_entry_copy.local_information = new_viewpoint_information;
  new_path_entry_copy.local_objective = new_viewpoint_information;
  viewpoint_path->entries.emplace_back(std::move(new_path_entry_copy));
  viewpoint_path->acc_information += new_path_entry_copy.local_information;
  viewpoint_path->acc_motion_distance += new_path_entry_copy.local_motion_distance;
  viewpoint_path->acc_objective += new_path_entry_copy.local_objective;
  viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
  viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
  viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;
}

void ViewpointPlanner::addStereoViewpointPathEntryWithoutLock(
        ViewpointPath *viewpoint_path, ViewpointPathComputationData *comp_data,
        const ViewpointPathEntry &first_path_entry, const ViewpointPathEntry &second_path_entry,
        const bool add_second_entry /*= true*/) {
  addViewpointPathEntryWithoutLock(viewpoint_path, comp_data, first_path_entry);
  if (add_second_entry) {
    addViewpointPathEntryWithoutLock(viewpoint_path, comp_data, second_path_entry);
//    ViewpointPathEntry second_path_entry_copy = second_path_entry;
//    second_path_entry_copy.local_information = 0;
//    viewpoint_path->entries.emplace_back(std::move(second_path_entry_copy));
//    viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
//    viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
//    viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;
  }

//  const ViewpointEntry& first_viewpoint_entry = viewpoint_entries_[first_path_entry.viewpoint_index];
//  const ViewpointEntry& second_viewpoint_entry = viewpoint_entries_[second_path_entry.viewpoint_index];
//  const auto overlap_set = bh::computeSetIntersection(first_viewpoint_entry.voxel_set, second_viewpoint_entry.voxel_set);
//  FloatType new_viewpoint_information = 0;
//  for (const VoxelWithInformation& vi : overlap_set) {
////    FloatType observation_information = options_.viewpoint_information_factor * vi.information;
//    const auto first_vi_it = first_viewpoint_entry.voxel_set.find(vi);
//    const auto second_vi_it = second_viewpoint_entry.voxel_set.find(vi);
//    const FloatType observation_information = options_.viewpoint_information_factor * std::max<FloatType>(
//        first_vi_it->information, second_vi_it->information);
//    FloatType novel_observation_information;
//    auto it = viewpoint_path->observed_voxel_map.find(vi.voxel);
//    if (it == viewpoint_path->observed_voxel_map.end()) {
//      viewpoint_path->observed_voxel_map.emplace(vi.voxel, observation_information);
//      novel_observation_information = observation_information;
//    }
//    else {
//      const WeightType voxel_weight = vi.voxel->getObject()->weight;
//      novel_observation_information = std::min(observation_information, voxel_weight - it->second);
//      AIT_ASSERT(it->second <= voxel_weight);
//      if (it->second <= voxel_weight) {
//        it->second += observation_information;
//        if (it->second > voxel_weight) {
//          it->second = voxel_weight;
//        }
//      }
//    }
//    AIT_ASSERT(novel_observation_information >= 0);
//    new_viewpoint_information += novel_observation_information;
//  }
//  AIT_PRINT_VALUE(overlap_set.size());
//  AIT_PRINT_VALUE(new_viewpoint_information);
//  viewpoint_path->acc_information += new_viewpoint_information;
//  ViewpointPathEntry first_path_entry_copy = first_path_entry;
//  first_path_entry_copy.local_information = new_viewpoint_information;
//  viewpoint_path->entries.emplace_back(std::move(first_path_entry_copy));
//  viewpoint_path->acc_motion_distance += first_path_entry_copy.local_motion_distance;
//  viewpoint_path->acc_objective += first_path_entry_copy.local_objective;
//  viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
//  viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
//  viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;
//  if (add_second_entry) {
//    viewpoint_path->entries.emplace_back(std::move(second_path_entry));
//    viewpoint_path->entries.back().acc_information = viewpoint_path->acc_information;
//    viewpoint_path->entries.back().acc_motion_distance = viewpoint_path->acc_motion_distance;
//    viewpoint_path->entries.back().acc_objective = viewpoint_path->acc_objective;
//  }
}

void ViewpointPlanner::initializeViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  std::unique_lock<std::mutex> lock(mutex_);
  viewpoint_paths_initialized_ = true;
  if (viewpoint_paths_.size() > viewpoint_graph_.size()) {
    // Make sure we don't expect too many viewpoints
    viewpoint_paths_.resize(viewpoint_graph_.size());
    viewpoint_paths_data_.resize(viewpoint_graph_.size());
  }
#pragma omp parallel for
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    initializeViewpointPathInformations(&viewpoint_path, &comp_data);
  }
}

void ViewpointPlanner::findInitialViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  if (viewpoint_paths_.size() > viewpoint_graph_.size()) {
    // Make sure we don't expect too many viewpoints
    viewpoint_paths_.resize(viewpoint_graph_.size());
    viewpoint_paths_data_.resize(viewpoint_graph_.size());
  }

  std::vector<ViewpointPathEntry> best_path_entries;

  // Find the overall best viewpoints with some min distance as starting points
  std::vector<ViewpointEntryIndex> entries;
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    entries.push_back(it.node());
  }
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPathEntry best_path_entry;
    // Make sure there are viewpoint entries left (otherwise just take one that was already selected)
    if (entries.empty()) {
      best_path_entry = best_path_entries.back();
    }
    else {
      for (ViewpointEntryIndex viewpoint_index : entries) {
        const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
        FloatType new_information = viewpoint_entry.total_information;
        FloatType new_objective = new_information - alpha;
        if (new_objective > best_path_entry.local_objective) {
          best_path_entry.viewpoint_index = viewpoint_index;
          best_path_entry.local_information = new_information;
          best_path_entry.local_motion_distance = 0;
          best_path_entry.local_objective = new_objective;
          best_path_entry.acc_information = new_information;
          best_path_entry.acc_motion_distance = 0;
          best_path_entry.acc_objective = new_objective;
          best_path_entry.viewpoint = viewpoint_entry.viewpoint;
        }
      }
    }
    best_path_entries.push_back(std::move(best_path_entry));

    // Remove all viewpoints that are too close to the found best one
    const ViewpointEntry& viewpoint = viewpoint_entries_[best_path_entry.viewpoint_index];
    for (auto it = entries.begin(); it != entries.end();) {
      const ViewpointEntry& other_viewpoint = viewpoint_entries_[*it];
      const FloatType distance = (other_viewpoint.viewpoint.pose().getWorldPosition() - viewpoint.viewpoint.pose().getWorldPosition()).norm();
      if (distance < options_.viewpoint_path_initial_distance) {
        it = entries.erase(it);
      }
      else {
        ++it;
      }
    }
  }

  std::unique_lock<std::mutex> lock(mutex_);
  for (auto it = viewpoint_paths_.begin(); it != viewpoint_paths_.end(); ++it) {
    AIT_ASSERT(it->entries.empty());
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[it - viewpoint_paths_.begin()];
    const ViewpointPathEntry& best_path_entry = best_path_entries[it - viewpoint_paths_.begin()];
    const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
    it->acc_information = best_path_entry.acc_information;
    it->acc_motion_distance = best_path_entry.acc_motion_distance;
    it->acc_objective = best_path_entry.acc_objective;
    it->entries.emplace_back(std::move(best_path_entry));
    AIT_ASSERT(it->observed_voxel_map.empty());
    for (const VoxelWithInformation& voxel_with_information : best_viewpoint_entry.voxel_set) {
      it->observed_voxel_map.emplace(
          voxel_with_information.voxel,
          options_.viewpoint_information_factor * voxel_with_information.information);
    }
//    it->observed_voxel_set.insert(best_viewpoint_entry.voxel_set.cbegin(), best_viewpoint_entry.voxel_set.cend());
    comp_data.num_connected_entries = 1;
    std::cout << "Initial viewpoint [" << (it - viewpoint_paths_.begin())
        << "]: acc_objective=" << it->acc_objective << std::endl;
  }
  lock.unlock();
}

void ViewpointPlanner::initializeViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
  if (!options_.ignore_real_observed_voxels) {
    for (std::size_t i = 0; i < num_real_viewpoints_; ++i) {
      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[i];
      for (const VoxelWithInformation& vi : viewpoint_entry.voxel_set) {
        const FloatType observation_information = options_.viewpoint_information_factor * vi.information;
        viewpoint_path->observed_voxel_map.emplace(vi.voxel, observation_information);
      }
//      viewpoint_path->observed_voxel_set.insert(viewpoint_entry.voxel_set.cbegin(), viewpoint_entry.voxel_set.cend());
    }
  }
  comp_data->sorted_new_informations.clear();
  for (auto it = viewpoint_graph_.begin(); it != viewpoint_graph_.end(); ++it) {
    const ViewpointEntryIndex viewpoint_index = it.node();
    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];
    comp_data->sorted_new_informations.push_back(std::make_tuple(viewpoint_index, viewpoint_entry.total_information, true));
  }
  std::sort(comp_data->sorted_new_informations.begin(), comp_data->sorted_new_informations.end(),
      [](const std::tuple<ViewpointEntryIndex, FloatType, bool>& a, const std::tuple<ViewpointEntryIndex, FloatType, bool>& b) {
        return std::get<1>(a) < std::get<1>(b);
  });
}

void ViewpointPlanner::updateViewpointPathInformations(ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data) {
#if !AIT_RELEASE
  AIT_ASSERT(!comp_data->sorted_new_informations.empty());
#endif
//  // Test code for checking the lazy update scheme
//  for (auto it = comp_data->sorted_new_informations.begin(); it != comp_data->sorted_new_informations.end(); ++it) {
//    const FloatType new_information = computeNewInformation(viewpoint_path, it->first);
//    it->second = new_information;
//  }
//  std::sort(comp_data->sorted_new_informations.begin(), comp_data->sorted_new_informations.end(),
//      [](const std::pair<ViewpointEntryIndex, FloatType>& a, const std::pair<ViewpointEntryIndex, FloatType>& b) {
//        return a.second < b.second;
//  });
//  // End of test code

  // Updating information without considering triangulation
  // TODO: This would be better to do with a binary heap
  // Our function is sub-modular so we can lazily update the best entries
  std::size_t recompute_count = 0;
  bool change_occured;
  do {
    change_occured = false;
    auto best_it = comp_data->sorted_new_informations.rbegin();
    FloatType new_information;
    if (std::get<2>(*best_it)) {
//      new_information = computeNewInformation(*viewpoint_path, *comp_data, std::get<0>(*best_it));
      new_information = evaluateNovelViewpointInformation(*viewpoint_path, *comp_data, std::get<0>(*best_it));
    }
    else {
      new_information = 0;
    }
    std::get<1>(*best_it) = new_information;
    ++recompute_count;
    for (auto it = comp_data->sorted_new_informations.rbegin() + 1; it != comp_data->sorted_new_informations.rend(); ++it) {
      if (std::get<1>(*it) > std::get<1>(*(it-1))) {
        change_occured = true;
        // Swap with higher entry if update gives better information score
        swap(*it, *(it - 1));
      }
      else {
        break;
      }
    }
  } while (change_occured);
  std::cout << "Recomputed " << recompute_count << " of " << comp_data->sorted_new_informations.size() << " viewpoints" << std::endl;
}

bool ViewpointPlanner::findNextViewpointPathEntries(const FloatType alpha, const FloatType beta) {
  if (!viewpoint_paths_initialized_) {
    initializeViewpointPathEntries(alpha, beta);
  }
//  if (viewpoint_paths_.front().entries.empty()) {
//    findInitialViewpointPathEntries(alpha, beta);
//    std::vector<bool> results(viewpoint_paths_.size());
//#pragma omp parallel for
//    for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
//      ViewpointPath& viewpoint_path = viewpoint_paths_[i];
//      ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
//      initializeViewpointPathInformations(&viewpoint_path, &comp_data);
//      if (options_.viewpoint_generate_stereo_pairs) {
//        // Try to find a matching stereo pair. Otherwise discard.
//        const ViewpointPathEntry& path_entry = viewpoint_path.entries.front();
//        ViewpointEntryIndex stereo_viewpoint_index;
//        bool stereo_viewpoint_already_in_path;
//        std::tie(stereo_viewpoint_already_in_path, stereo_viewpoint_index) = findMatchingStereoViewpointWithoutLock(
//            &viewpoint_path, &comp_data, path_entry);
//        if (!stereo_viewpoint_already_in_path && stereo_viewpoint_index == (ViewpointEntryIndex)-1) {
//          results[i] = false;
//        }
//        else {
//          results[i] = true;
//        }
//
//        if (results[i] && !stereo_viewpoint_already_in_path) {
//          // Add stereo viewpoint entry to path
//          const ViewpointEntry& stereo_viewpoint_entry = viewpoint_entries_[stereo_viewpoint_index];
//          ViewpointPathEntry stereo_path_entry;
//          stereo_path_entry.viewpoint_index = stereo_viewpoint_index;
//          stereo_path_entry.local_information = computeNewInformation(viewpoint_path, comp_data, stereo_viewpoint_index);
//          stereo_path_entry.local_motion_distance = 0;
//          stereo_path_entry.local_objective = stereo_path_entry.local_information;
//          stereo_path_entry.viewpoint = stereo_viewpoint_entry.viewpoint;
//          addViewpointPathEntryWithoutLock(&viewpoint_path, &comp_data, stereo_path_entry);
//        }
//      }
//    }
//    if (options_.viewpoint_generate_stereo_pairs) {
//      for (bool result : results) {
//        if (!result) {
//          std::cout << "Not all initialized viewpoints could be matched with a stereo pair" << std::endl;
//          viewpoint_paths_.clear();
//          viewpoint_paths_data_.clear();
//          return false;
//        }
//      }
//    }
//    reportViewpointPathsStats();
//    return true;
//  }

  std::size_t changes = 0;
#pragma omp parallel for \
  reduction(+:changes)
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    const bool randomize = i > 0;
    changes += findNextViewpointPathEntry(&viewpoint_path, &comp_data, randomize, alpha, beta);
  }
  reportViewpointPathsStats();

  std::cout << "changes=" << changes << ", alpha=" << alpha << ", beta=" << beta << std::endl;
  if (changes == 0) {
    return false;
  }
  else {
    return true;
  }
}

bool ViewpointPlanner::findNextViewpointPathEntries() {
  return findNextViewpointPathEntries(options_.objective_parameter_alpha, options_.objective_parameter_beta);
}

std::pair<ViewpointPlanner::ViewpointEntryIndex, ViewpointPlanner::FloatType> ViewpointPlanner::getBestNextViewpoint(
      const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const bool randomize) const {
  if (randomize) {
    // TODO: Make as parameters
    const std::size_t num_of_good_viewpoints_to_sample_from = 100;
    auto it = random_.sampleDiscreteWeighted(
        comp_data.sorted_new_informations.cend() - num_of_good_viewpoints_to_sample_from,
        comp_data.sorted_new_informations.cend(),
        [](const std::tuple<ViewpointEntryIndex, FloatType, bool>& entry) {
      const FloatType information = std::get<1>(entry);
      return information;
    });
    if (it == comp_data.sorted_new_informations.cend()) {
      it = comp_data.sorted_new_informations.cend() - 1;
    }
    return std::make_pair(std::get<0>(*it), std::get<1>(*it));
  }
  else {
    return std::make_pair(std::get<0>(comp_data.sorted_new_informations.back()),
                          std::get<1>(comp_data.sorted_new_informations.back()));
  }
}

// TODO: Upper bound is wrong when voxels are triangulated
ViewpointPlanner::FloatType ViewpointPlanner::computeViewpointPathInformationUpperBound(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data, const std::size_t max_num_viewpoints) const {
  FloatType information_upper_bound = viewpoint_path.acc_information;
  const std::size_t num_viewpoints = std::min(max_num_viewpoints, comp_data.sorted_new_informations.size());
  for (auto it = comp_data.sorted_new_informations.rbegin(); it != comp_data.sorted_new_informations.rbegin() + num_viewpoints; ++it) {
    const FloatType information = std::get<1>(*it);
    information_upper_bound += information;
  }
  return information_upper_bound;
}

std::pair<bool, ViewpointPlanner::ViewpointEntryIndex> ViewpointPlanner::findMatchingStereoViewpointWithoutLock(
    const ViewpointPath& viewpoint_path,
    const ViewpointPathComputationData& comp_data,
    const ViewpointEntryIndex& viewpoint_index,
    const bool ignore_sparse_matching,
    const bool ignore_graph_component) {
  try {
    const bool verbose = true;

    const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];

    // Compute center position of voxels with information
    Vector3 voxel_center = Vector3::Zero();
    FloatType total_weight = 0;
    // TODO
  //  std::accumulate(viewpoint_entry.voxel_set.begin(), viewpoint_entry.voxel_set.end(), voxel_center,
  //      [](Vector3& vector, const VoxelWithInformation& vi) {
  //    vector += vi.voxel->getBoundingBox().getCenter();
  //    return vector;
  //  });
    for (const VoxelWithInformation& vi : viewpoint_entry.voxel_set) {
      const FloatType information = vi.information;
      voxel_center += information * vi.voxel->getBoundingBox().getCenter();
      total_weight += information;
  //    voxel_center += vi.voxel->getObject()->weight * vi.voxel->getBoundingBox().getCenter();
  //    total_weight += vi.voxel->getObject()->weight;
    }
  //  voxel_center /= viewpoint_entry.voxel_set.size();
      voxel_center /= total_weight;

    // Compute minimum and maximum stereo baseline
    const Pose& pose = viewpoint_entry.viewpoint.pose();
    const FloatType dist_to_voxel_sq = (voxel_center - pose.getWorldPosition()).squaredNorm();
    const FloatType min_baseline_square = 4 * dist_to_voxel_sq * triangulation_min_sin_angle_square_;
    const FloatType max_baseline_square = 4 * dist_to_voxel_sq * triangulation_max_sin_angle_square_;
    std::cout << "min_baseline_square=" << min_baseline_square << std::endl;
    std::cout << "max_baseline_square=" << max_baseline_square << std::endl;

    const auto is_stereo_pair_lambda = [&](const ViewpointEntry& other_viewpoint, const bool ignore_angular_deviation) -> bool {
      const Pose& other_pose = other_viewpoint.viewpoint.pose();
      const Vector3 rel_pose_vector = other_pose.getWorldPosition() - pose.getWorldPosition();
      const Vector3 mid_point = (pose.getWorldPosition() + other_pose.getWorldPosition()) / 2;
      const Vector3 mid_point_to_voxel_center = voxel_center - mid_point;
      const Vector3 mid_point_to_voxel_center_norm = mid_point_to_voxel_center.normalized();
      const FloatType dist_deviation = mid_point_to_voxel_center_norm.dot(rel_pose_vector);
      const FloatType dist_deviation_ratio = std::abs(dist_deviation) / mid_point_to_voxel_center.norm();
      const Vector3 baseline_vector = rel_pose_vector - dist_deviation * mid_point_to_voxel_center_norm;
      const FloatType baseline_square = baseline_vector.squaredNorm();
  //      const FloatType other_dist_deviation_ratio = std::abs(other_dist_to_voxel_sq - dist_to_voxel_sq) / dist_to_voxel_sq;
      const FloatType angular_deviation = other_pose.quaternion().angularDistance(pose.quaternion());
  //    AIT_PRINT_VALUE(rel_pose_vector.transpose());
  //    AIT_PRINT_VALUE(mid_point_to_voxel_center_norm.transpose());
  //    AIT_PRINT_VALUE(dist_deviation);
  //    AIT_PRINT_VALUE(dist_deviation_ratio);
  //    AIT_PRINT_VALUE(baseline_square);
  //    AIT_PRINT_VALUE(angular_deviation);
      const bool is_dist_deviation_valid = dist_deviation_ratio <= options_.triangulation_max_dist_deviation_ratio;
      const bool is_baseline_valid = baseline_square >= min_baseline_square && baseline_square <= max_baseline_square;
      bool is_angular_deviation_valid = angular_deviation <= triangulation_max_angular_deviation_;
  //    AIT_PRINT_VALUE(is_dist_deviation_valid);
  //    AIT_PRINT_VALUE(is_angular_deviation_valid);
  //    AIT_PRINT_VALUE(is_baseline_valid);
      if (ignore_angular_deviation) {
        is_angular_deviation_valid = true;
      }
      if (is_dist_deviation_valid && is_baseline_valid && is_angular_deviation_valid) {
        return true;
      }
      else {
        return false;
      }
    };

    const std::vector<std::size_t>& component = getConnectedComponents().first;

    // Check if a matching viewpoint already exists
    // TODO: Remove index output.
  //  std::size_t i = 0;
    for (const ViewpointPathEntry& other_path_entry : viewpoint_path.entries) {
  //    std::cout << "i=" << i << std::endl;
  //    ++i;
      // TODO: Check shouldn't be necessary
      if (other_path_entry.viewpoint_index != viewpoint_index) {
        const ViewpointEntryIndex& other_index = other_path_entry.viewpoint_index;
  //      std::cout << "other_index=" << other_index << std::endl;
        const ViewpointEntry& other_viewpoint = viewpoint_entries_[other_index];
        const bool ignore_angular_deviation = false;
        if (is_stereo_pair_lambda(other_viewpoint, ignore_angular_deviation)) {
          // Compute overlap information
          const VoxelWithInformationSet overlap_set = bh::computeSetIntersection(viewpoint_entry.voxel_set, other_viewpoint.voxel_set);
          const FloatType voxel_overlap_ratio = overlap_set.size() / (FloatType)viewpoint_entry.voxel_set.size();
          const FloatType overlap_information = computeInformationScore(other_viewpoint.viewpoint, overlap_set.begin(), overlap_set.end());
          const FloatType information_overlap_ratio = overlap_information / viewpoint_entry.total_information;
          const bool sparse_matchable = ignore_sparse_matching || isSparseMatchable(viewpoint_index, other_index);
          AIT_PRINT_VALUE(voxel_overlap_ratio);
          AIT_PRINT_VALUE(information_overlap_ratio);
          if (ignore_graph_component || (component[other_index] == component[viewpoint_index]
              && voxel_overlap_ratio >= options_.triangulation_min_voxel_overlap_ratio
              && information_overlap_ratio >= options_.triangulation_min_information_overlap_ratio
              && sparse_matchable)) {
            return std::make_pair(true, other_index);
          }
        }
      }
    }

    // Find viewpoint in baseline range with highest information overlap for same viewing direction
    // TODO: Should use a radius search
    const std::size_t knn = options_.triangulation_knn;
    static std::vector<ViewpointANN::IndexType> knn_indices;
    static std::vector<ViewpointANN::DistanceType> knn_distances;
    knn_indices.resize(knn);
    knn_distances.resize(knn);
    viewpoint_ann_.knnSearch(pose.getWorldPosition(), knn, &knn_indices, &knn_distances);
    ViewpointEntryIndex best_index = (ViewpointEntryIndex)-1;
    FloatType best_overlap_information = std::numeric_limits<FloatType>::lowest();
    for (ViewpointANN::IndexType other_index : knn_indices) {
      if (!ignore_graph_component && component[other_index] != component[viewpoint_index]) {
        continue;
      }
      const ViewpointEntry& other_viewpoint = viewpoint_entries_[other_index];
      const bool ignore_angular_deviation = true;
      if (is_stereo_pair_lambda(other_viewpoint, ignore_angular_deviation)) {
        // Compute viewpoint for new pose
        const Vector3 look_at_direction = voxel_center - other_viewpoint.viewpoint.pose().translation();
        const Quaternion new_quaternion = bh::getZLookAtQuaternion(look_at_direction, Vector3::UnitZ());
        const Pose new_pose = Pose::createFromImageToWorldTransformation(
            other_viewpoint.viewpoint.pose().translation(),
            new_quaternion);
  //      const Pose new_pose = Pose::createFromImageToWorldTransformation(
  //          other_viewpoint.viewpoint.pose().translation(),
  //          viewpoint_entry.viewpoint.pose().quaternion());
        const Viewpoint new_viewpoint = getVirtualViewpoint(new_pose);
        // Check if sparse matchable
        const bool sparse_matchable = ignore_sparse_matching || isSparseMatchable(viewpoint_entry.viewpoint, new_viewpoint);
        if (!sparse_matchable) {
          continue;
        }
        // Compute overlap information by raycasting from new viewpoint
        std::pair<VoxelWithInformationSet, FloatType> raycast_result =
            getRaycastHitVoxelsWithInformationScore(new_viewpoint);
        VoxelWithInformationSet& new_voxel_set = raycast_result.first;
        FloatType overlap_information = 0;
        for (const VoxelWithInformation& vi : new_voxel_set) {
          const auto it = viewpoint_entry.voxel_set.find(vi);
          if (it != viewpoint_entry.voxel_set.end()) {
            overlap_information += std::min(it->information, vi.information);
          }
        }
        if (overlap_information > best_overlap_information) {
          best_index = other_index;
          best_overlap_information = overlap_information;
        }
  //      const VoxelWithInformationSet overlap_set = bh::computeSetIntersection(viewpoint_entry.voxel_set, other_viewpoint.voxel_set);
  //      const FloatType overlap_information = computeInformationScore(other_viewpoint.viewpoint, overlap_set.begin(), overlap_set.end());
  //      if (component[other_index] == component[viewpoint_index] && overlap_information > best_overlap_information) {
  //        best_index = other_index;
  //        best_overlap_information = overlap_information;
  //      }
      }
    }

    if (best_index == (ViewpointEntryIndex)-1) {
      if (verbose) {
        std::cout << "Could not find any stereo viewpoint in the right baseline and distance range" << std::endl;
      }
      return std::make_pair(false, (ViewpointEntryIndex)-1);
    }

    const FloatType best_overlap_ratio = best_overlap_information / viewpoint_entry.total_information;
    AIT_PRINT_VALUE(best_index);
    AIT_PRINT_VALUE(best_overlap_ratio);
    std::size_t stereo_viewpoint_index = best_index;
    // TODO: Always create new viewpoint for stereo pair?
  //  if (best_overlap_ratio < options_.triangulation_min_overlap_ratio) {
    if (true) {
      // Add viewpoint candidate with same translation as best found stereo viewpoint and same orientation as reference viewpoint.
      const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_index];
      // Compute viewpoint for new pose
      const Vector3 look_at_direction = voxel_center - best_viewpoint_entry.viewpoint.pose().translation();
      const Quaternion new_quaternion = bh::getZLookAtQuaternion(look_at_direction, Vector3::UnitZ());
      const Pose new_pose = Pose::createFromImageToWorldTransformation(
          best_viewpoint_entry.viewpoint.pose().translation(),
          new_quaternion);
  //    const Pose new_pose = Pose::createFromImageToWorldTransformation(
  //        best_viewpoint_entry.viewpoint.pose().translation(),
  //        viewpoint_entry.viewpoint.pose().quaternion());
      const Viewpoint new_viewpoint = getVirtualViewpoint(new_pose);
      const bool ignore_voxels_with_zero_information = true;
      // Compute raycast for new viewpoint
      std::pair<VoxelWithInformationSet, FloatType> raycast_result =
          getRaycastHitVoxelsWithInformationScore(new_viewpoint, ignore_voxels_with_zero_information);
      VoxelWithInformationSet& new_voxel_set = raycast_result.first;
      const FloatType new_total_information = raycast_result.second;
      const VoxelWithInformationSet overlap_set = bh::computeSetIntersection(viewpoint_entry.voxel_set, new_voxel_set);
      const FloatType voxel_overlap_ratio = overlap_set.size() / (FloatType)viewpoint_entry.voxel_set.size();
      const FloatType first_total_information = computeInformationScore(viewpoint_entry.viewpoint, viewpoint_entry.voxel_set.begin(), viewpoint_entry.voxel_set.end());
      const FloatType second_total_information = computeInformationScore(new_viewpoint, new_voxel_set.begin(), new_voxel_set.end());
      const FloatType overlap_information = computeInformationScore(new_viewpoint, overlap_set.begin(), overlap_set.end());
      const FloatType information_overlap_ratio = overlap_information / viewpoint_entry.total_information;
      AIT_PRINT_VALUE(viewpoint_entry.voxel_set.size());
      AIT_PRINT_VALUE(viewpoint_entry.total_information);
      AIT_PRINT_VALUE(new_total_information);
      AIT_PRINT_VALUE(new_voxel_set.size());
      AIT_PRINT_VALUE(overlap_set.size());
      AIT_PRINT_VALUE(voxel_overlap_ratio);
      AIT_PRINT_VALUE(first_total_information);
      AIT_PRINT_VALUE(second_total_information);
      AIT_PRINT_VALUE(overlap_information);
      AIT_PRINT_VALUE(information_overlap_ratio);
      if (voxel_overlap_ratio < options_.triangulation_min_voxel_overlap_ratio
          || information_overlap_ratio < options_.triangulation_min_information_overlap_ratio) {
        if (verbose) {
          std::cout << "Could not find any stereo viewpoint with enough overlap" << std::endl;
        }
        return std::make_pair(false, (ViewpointEntryIndex)-1);
      }
      // TODO: Require minimum overlap?
  //    if (overlap_ratio >= options_.triangulation_min_overlap_ratio) {
      if (true) {
        if (verbose) {
          std::cout << "Adding stereo viewpoint to graph" << std::endl;
        }
        ViewpointEntry new_viewpoint_entry(new_viewpoint, new_total_information, std::move(new_voxel_set));
        stereo_viewpoint_index = addViewpointEntryWithoutLock(std::move(new_viewpoint_entry));

        if (verbose) {
          std::cout << "Adding motion to stereo viewpoint to graph" << std::endl;
        }
        // Make sure stereo viewpoints can be connected by replicating the connections of best_index.
        // We already know that the reference view and best_index belong to the same component.
        auto edges = viewpoint_graph_.getEdgesByNode(best_index);
        for (auto it = edges.begin(); it != edges.end(); ++it) {
          ViewpointMotion motion = getViewpointMotion(best_index, it.targetNode());
          std::vector<ViewpointEntryIndex> new_viewpoint_indices = motion.viewpointIndices();
          new_viewpoint_indices.front() = stereo_viewpoint_index;
          addViewpointMotion(ViewpointMotion(new_viewpoint_indices, motion.se3Motions()));
        }
      }
    }
    AIT_PRINT_VALUE(stereo_viewpoint_index);

  //  if (stereo_path_entry.local_objective <= 0) {
  //    if (verbose) {
  //      std::cout << "No improvement in objective in stereo viewpoint" << std::endl;
  //    }
  //    return false;
  //  }

    // If the overlap ratio was not above the threshold we could still have selected a stereo viewpoint
    // that is already in the path.
     const auto it = std::find_if(viewpoint_path.entries.begin(), viewpoint_path.entries.end(),
        [&](const ViewpointPathEntry& entry) {
       return entry.viewpoint_index == stereo_viewpoint_index;
     });
     const bool stereo_viewpoint_already_in_path = it != viewpoint_path.entries.end();

    return std::make_pair(stereo_viewpoint_already_in_path, stereo_viewpoint_index);
  }

  // TODO: Should be a exception for raycast
  catch (const ait::Error& err) {
    std::cout << "Raycast failed: " << err.what() << std::endl;
    return std::make_pair(false, (ViewpointEntryIndex)-1);
  }
}

bool ViewpointPlanner::findNextViewpointPathEntry(
    ViewpointPath* viewpoint_path, ViewpointPathComputationData* comp_data,
    const bool randomize, const FloatType alpha, const FloatType beta) {
  const bool verbose = true;

  // TODO: The mutex should be locked when updating viewpoint information
  updateViewpointPathInformations(viewpoint_path, comp_data);
  ViewpointEntryIndex new_viewpoint_index;
  FloatType new_information;
  std::tie(new_viewpoint_index, new_information) = getBestNextViewpoint(*viewpoint_path, *comp_data, randomize);

  ViewpointPathEntry best_path_entry;
  best_path_entry.viewpoint_index = new_viewpoint_index;
  best_path_entry.local_information = new_information;
  best_path_entry.local_motion_distance = 0;
  best_path_entry.local_objective = best_path_entry.local_information;
  best_path_entry.viewpoint = viewpoint_entries_[new_viewpoint_index].viewpoint;

//  ViewpointEntryIndex from_index = viewpoint_path->entries.back().viewpoint_index;
//  // Run through the neighbors and select the best next viewpoint
////  for (const auto& edge : viewpoint_graph_.getEdgesByNode(from_index)) {
//  //    ViewpointEntryIndex to_index = edge.first;
//  for (const ViewpointEntryIndex to_index : viewpoint_graph_) {
//    const ViewpointEntry& to_viewpoint = viewpoint_entries_[to_index];
//    VoxelWithInformationSet difference_set = bh::computeSetDifference(to_viewpoint.voxel_set, viewpoint_path->observed_voxel_set);
////    std::cout << "  j=" << j << ", next_node.voxel_set.size()= " << next_node.voxel_set.size() << std::endl;
////    std::cout << "  j=" << j << ", difference_set.size()= " << difference_set.size() << std::endl;
//    FloatType new_information = std::accumulate(difference_set.cbegin(), difference_set.cend(),
//        FloatType { 0 }, [](const FloatType& value, const VoxelWithInformation& voxel) {
//          return value + voxel.information;
//    });
////      if (verbose) {
////        std::cout << "difference_set.size()=" << difference_set.size() <<
////            ", new_information=" << new_information << std::endl;
////      }
//    float motion_distance = viewpoint_graph_.getWeight(from_index, to_index);
//    FloatType new_objective = new_information - alpha - beta * motion_distance * motion_distance;
////    std::cout << "  j=" << j << ", new_information=" << new_information << std::endl;
//    if (verbose) {
//      std::cout << "to_index=" << to_index
//          << ", voxel_set.size()=" << to_viewpoint.voxel_set.size()
//          << ", new_information=" << new_information
//          << ", motion_distance=" << motion_distance
//          << ", new_objective=" << new_objective << std::endl;
//    }
//    if (new_objective > best_path_entry.local_objective) {
//      best_path_entry.viewpoint_index = to_index;
//      best_path_entry.local_information = new_information;
//      best_path_entry.local_motion_distance = motion_distance;
//      best_path_entry.local_objective = new_objective;
//    }
//  }

  if (best_path_entry.viewpoint_index == (ViewpointEntryIndex)-1) {
    return false;
  }

  if (best_path_entry.local_objective <= 0) {
    if (verbose) {
      std::cout << "No improvement in objective" << std::endl;
    }
    return false;
  }

//  if (verbose) {
//      const ViewpointEntry& best_viewpoint_entry = viewpoint_entries_[best_path_entry.viewpoint_index];
//    // TODO: remove this set difference (duplicate anyway)
//    VoxelWithInformationSet difference_set = bh::computeSetDifference(best_viewpoint_entry.voxel_set, viewpoint_path->observed_voxel_set);
//    std::cout << "Found viewpoint with local_information=" << best_path_entry.local_information <<
//        ", total_information=" << best_viewpoint_entry.total_information <<
//        ", total_voxels=" << best_viewpoint_entry.voxel_set.size() <<
//        ", new_voxels=" << difference_set.size() << std::endl;
//  }

  std::unique_lock<std::mutex> lock(mutex_);

//  if (verbose) {
//    for (auto entry : viewpoint_path->entries) {
//      std::cout << "entry: " << entry.viewpoint_index << std::endl;
//    }
//    std::cout << "best entry: " << best_path_entry.viewpoint_index << std::endl;
//  }

  const bool is_valid_path_entry = isValidViewpointPathEntry(*viewpoint_path, *comp_data, best_path_entry.viewpoint_index);
  if (!is_valid_path_entry) {
    if (verbose) {
      std::cout << "Selected viewpoint is not a valid path entry. Invalidating: " << best_path_entry.viewpoint_index << std::endl;
    }
    std::get<2>(*comp_data->sorted_new_informations.rbegin()) = false;
    return false;
  }

  ViewpointEntryIndex stereo_viewpoint_index = (ViewpointEntryIndex)-1;
  bool stereo_viewpoint_already_in_path = false;
  if (options_.viewpoint_generate_stereo_pairs) {
    // Try to find a matching stereo pair. Otherwise discard.
    std::tie(stereo_viewpoint_already_in_path, stereo_viewpoint_index) = findMatchingStereoViewpointWithoutLock(
        *viewpoint_path, *comp_data, best_path_entry.viewpoint_index);
    if (!stereo_viewpoint_already_in_path && stereo_viewpoint_index == (ViewpointEntryIndex)-1) {
      if (verbose) {
        std::cout << "Could not find any usable stereo viewpoint for viewpoint " << best_path_entry.viewpoint_index << std::endl;
      }
      // Mark viewpoint as invalid to prevent use in the future
      std::get<2>(*comp_data->sorted_new_informations.rbegin()) = false;
      return false;
    }
    const bool ignore_if_already_on_path = true;
    const bool is_valid_stereo_path_entry = isValidViewpointPathEntry(
            *viewpoint_path, *comp_data,
            stereo_viewpoint_index, ignore_if_already_on_path);
    if (!is_valid_stereo_path_entry) {
      if (verbose) {
        std::cout << "Matched stereo viewpoint is not a valid path entry. Invalidating: " << best_path_entry.viewpoint_index << std::endl;
      }
      std::get<2>(*comp_data->sorted_new_informations.rbegin()) = false;
      return false;
    }
  }

  // Mark viewpoint as invalid to prevent use in the future
  std::get<2>(*comp_data->sorted_new_informations.rbegin()) = false;

  if (options_.viewpoint_generate_stereo_pairs) {
    if (options_.dump_stereo_matching_images) {
      bool draw_lines = false;
      dumpSparseMatching(best_path_entry.viewpoint_index, stereo_viewpoint_index,
                              std::string("dump/path_sparse_points_") + std::to_string(viewpoint_path->entries.size())
                              + "_" + std::to_string(best_path_entry.viewpoint_index)
                              + "_" + std::to_string(stereo_viewpoint_index) + ".png",
                              draw_lines);
      draw_lines = true;
      dumpSparseMatching(best_path_entry.viewpoint_index, stereo_viewpoint_index,
                              std::string("dump/path_sparse_matching_") + std::to_string(viewpoint_path->entries.size())
                              + "_" + std::to_string(best_path_entry.viewpoint_index)
                              + "_" + std::to_string(stereo_viewpoint_index) + ".png",
                              draw_lines);
    }

    // Add stereo viewpoint entry to path
    ViewpointPathEntry stereo_path_entry;
    stereo_path_entry.viewpoint_index = stereo_viewpoint_index;
    stereo_path_entry.local_information = 0;
    //stereo_path_entry.local_information = computeNewInformation(*viewpoint_path, *comp_data, stereo_viewpoint_index);
    stereo_path_entry.local_motion_distance = 0;
    stereo_path_entry.local_objective = stereo_path_entry.local_information;
    stereo_path_entry.viewpoint = viewpoint_entries_[stereo_viewpoint_index].viewpoint;
    if (stereo_viewpoint_already_in_path) {
      const bool add_second_entry = false;
      addStereoViewpointPathEntryWithoutLock(viewpoint_path, comp_data,
                                             best_path_entry, stereo_path_entry, add_second_entry);
    }
    else {
      addStereoViewpointPathEntryWithoutLock(viewpoint_path, comp_data,
                                             best_path_entry, stereo_path_entry);
    }
  }
  else {
    addViewpointPathEntryWithoutLock(viewpoint_path, comp_data, best_path_entry);
  }

  lock.unlock();
//  if (verbose) {
//    std::cout << "new information: " <<  best_path_entry.information <<
//        ", information: " << best_viewpoint_entry.total_information << std::endl;
//  }

//  if (verbose) {
//    std::cout << "Path: " << std::endl;
//    for (std::size_t i = 0; i < viewpoint_path->entries.size(); ++i) {
//      const ViewpointPathEntry& path_entry = viewpoint_path->entries[i];
//      const ViewpointEntry& viewpoint_entry = viewpoint_entries_[path_entry.viewpoint_index];
//      std::cout << "  viewpoint_index: " << path_entry.viewpoint_index
//          << ", pose[" << i << "]: " << viewpoint_entry.viewpoint.pose().getWorldPosition().transpose()
//          << ", local_information: " << path_entry.local_information
//          << ", global_information: " << path_entry.acc_information
//          << ", local_motion_distance: " << path_entry.local_motion_distance
//          << ", global_motion_distance: " << path_entry.acc_motion_distance
//          << ", local_objective: " << path_entry.local_objective
//          << ", global_objective: " << path_entry.acc_objective << std::endl;
//    }
//  }

//  if (verbose) {
//    std::cout << "Done." << std::endl;
//  }

  return true;
}

bool ViewpointPlanner::isValidViewpointPathEntry(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntryIndex viewpoint_index,
    const bool ignore_if_already_on_path /*= false*/) const {
  const bool verbose = true;
  const ViewpointEntry& viewpoint_entry = viewpoint_entries_[viewpoint_index];

  // Check if viewpoint has the same component as the viewpoint path
  const std::vector<std::size_t>& component = getConnectedComponents().first;
  if (!viewpoint_path.entries.empty()) {
    if (component[viewpoint_index] != component[viewpoint_path.entries.front().viewpoint_index]) {
      if (verbose) {
        std::cout << "New viewpoint does not belong to the same connected component as the viewpoint path: " << viewpoint_index << std::endl;
      }
      return false;
    }
  }

  if (ignore_if_already_on_path) {
    return true;
  }

  // Check if viewpoint is already on path
  const bool is_already_on_path = std::find_if(viewpoint_path.entries.begin(), viewpoint_path.entries.end(),
      [&](const ViewpointPathEntry& entry) {
    return entry.viewpoint_index == viewpoint_index;
  }) != viewpoint_path.entries.end();
  if (is_already_on_path) {
    if (verbose) {
      std::cout << "Selected viewpoint that is already in the viewpoint path: " << viewpoint_index << std::endl;
    }
    return false;
  }

  // Check if viewpoint is too close to existing viewpoints
  if (std::find_if(viewpoint_path.entries.begin(), viewpoint_path.entries.end(),
      [&](const ViewpointPathEntry& entry) {
    const FloatType dist = (entry.viewpoint.pose().getWorldPosition()
            - viewpoint_entry.viewpoint.pose().getWorldPosition()).squaredNorm();
    return dist < options_.viewpoint_path_discard_dist_thres_square;
  }) != viewpoint_path.entries.end()) {
    if (verbose) {
      std::cout << "Selected viewpoint is too close to existing viewpoints in the viewpoint path: " << viewpoint_index << std::endl;
    }
    return false;
  }

  // Check if there is another viewpoint with a too close triangulation angle
  const Pose& pose = viewpoint_entry.viewpoint.pose();
  const Vector3 voxel_center = computeInformationVoxelCenter(viewpoint_entry);
  const FloatType dist_to_voxel_sq = (voxel_center - pose.getWorldPosition()).squaredNorm();
  const FloatType min_baseline_square = 4 * dist_to_voxel_sq * triangulation_min_sin_angle_square_;
  const FloatType max_baseline_square = 4 * dist_to_voxel_sq * triangulation_max_sin_angle_square_;
  AIT_PRINT_VALUE(min_baseline_square);
  AIT_PRINT_VALUE(max_baseline_square);
  if (std::find_if(viewpoint_path.entries.begin(), viewpoint_path.entries.end(),
      [&](const ViewpointPathEntry& entry) {
    const ViewpointEntry& other_viewpoint_entry = viewpoint_entries_[entry.viewpoint_index];
//    const Vector3 other_voxel_center = computeInformationVoxelCenter(other_viewpoint_entry);

    const Pose& other_pose = other_viewpoint_entry.viewpoint.pose();
    const Vector3 rel_pose_vector = other_pose.getWorldPosition() - pose.getWorldPosition();
    const Vector3 mid_point = (pose.getWorldPosition() + other_pose.getWorldPosition()) / 2;
    const Vector3 mid_point_to_voxel_center = voxel_center - mid_point;
    const Vector3 mid_point_to_voxel_center_norm = mid_point_to_voxel_center.normalized();
    const FloatType dist_deviation = mid_point_to_voxel_center_norm.dot(rel_pose_vector);
    const FloatType dist_deviation_ratio = std::abs(dist_deviation) / mid_point_to_voxel_center.norm();
    const Vector3 baseline_vector = rel_pose_vector - dist_deviation * mid_point_to_voxel_center_norm;
    const FloatType baseline_square = baseline_vector.squaredNorm();
    const FloatType angular_deviation = other_pose.quaternion().angularDistance(pose.quaternion());
    if (angular_deviation > triangulation_max_angular_deviation_) {
      return false;
    }
    if (dist_deviation_ratio > options_.triangulation_max_dist_deviation_ratio) {
      return false;
    }
    if (baseline_square >= min_baseline_square) {
      return false;
    }
    return true;
  }) != viewpoint_path.entries.end()) {
    if (verbose) {
      std::cout << "Selected viewpoint has a too small triangulation angle with an existing viewpoint: " << viewpoint_index << std::endl;
    }
    return false;
  }
  return true;
}

void ViewpointPlanner::reportViewpointPathsStats() const {
  for (std::size_t i = 0; i < viewpoint_paths_.size(); ++i) {
    const ViewpointPath& viewpoint_path = viewpoint_paths_[i];
    const ViewpointPathComputationData& comp_data = viewpoint_paths_data_[i];
    // Compute upper bound assuming we have all viewpoints
    const std::size_t max_num_viewpoints = viewpoint_path.entries.size();
    const FloatType information_upper_bound = computeViewpointPathInformationUpperBound(viewpoint_path, comp_data, max_num_viewpoints);
    std::cout << "Current information for branch " << i << ": " << viewpoint_path.acc_information
        << ", upper bound: " << information_upper_bound
        << ", ratio: " << (viewpoint_path.acc_information / information_upper_bound) << std::endl;
    const FloatType path_length = computeTourLength(viewpoint_path, viewpoint_path.order);
    std::cout << "Path length for branch " << i << ": " << path_length << std::endl;
  }
}
