//==================================================
// viewpoint_planner_graph.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 9, 2017
//==================================================

#include "viewpoint_planner.h"
#include <boost/graph/connected_components.hpp>

ViewpointPlanner::ViewpointEntryIndex ViewpointPlanner::addViewpointEntryWithoutLock(ViewpointEntry&& viewpoint_entry) {
  const ViewpointEntryIndex viewpoint_index = viewpoint_entries_.size();
  const Vector3 viewpoint_position = viewpoint_entry.viewpoint.pose().getWorldPosition();
//  std::cout << "Adding position to ANN index" << std::endl;
  viewpoint_ann_.addPoint(viewpoint_position);
//  std::cout << "Putting viewpoint entry into array" << std::endl;
  if (viewpoint_entries_.capacity() == viewpoint_entries_.size()) {
    std::cout << "Increasing capacity for viewpoint entries" << std::endl;
    const size_t new_viewpoint_entries_capacity = (size_t)std::ceil(1.25 * viewpoint_entries_.size());
    viewpoint_entries_.reserve(new_viewpoint_entries_capacity);
  }
  viewpoint_entries_.emplace_back(std::move(viewpoint_entry));
//  std::cout << "Adding node to viewpoint graph" << std::endl;
  viewpoint_graph_.addNode(viewpoint_index);
  viewpoint_graph_components_valid_ = false;
#if !AIT_RELEASE
  AIT_ASSERT(viewpoint_count_grid_.isInsideGrid(viewpoint_position));
#endif
  viewpoint_count_grid_(viewpoint_position) += 1;
  viewpoint_grid_indices_.push_back(viewpoint_count_grid_.getGridIndices(viewpoint_position));
  // Check if viewpoint sampling distribution should be updated
  const size_t increase_since_last_update = viewpoint_entries_.size() - viewpoint_sampling_distribution_update_size_;
  const FloatType relative_increase_since_last_update = increase_since_last_update / FloatType(viewpoint_entries_.size());
  if (viewpoint_entries_.size() >= num_real_viewpoints_
      && relative_increase_since_last_update >= FloatType(0.05)) {
    std::cout << "Updating viewpoint sampling distribution" << std::endl;
    updateViewpointSamplingDistribution();
    viewpoint_sampling_distribution_update_size_ = viewpoint_entries_.size();
  }
  return viewpoint_index;
}

void ViewpointPlanner::updateViewpointSamplingDistribution() {
  std::vector<FloatType> weights;
  weights.reserve(viewpoint_entries_.size());
  const auto grid_count_lambda = [&](const ViewpointEntry& entry) {
    const Vector3 viewpoint_position = entry.viewpoint.pose().getWorldPosition();
    return viewpoint_count_grid_(viewpoint_position);
  };
  const auto max_it = std::max_element(viewpoint_entries_.begin() + num_real_viewpoints_, viewpoint_entries_.end(),
                                       [&] (const ViewpointEntry& entry1, const ViewpointEntry& entry2) {
    return grid_count_lambda(entry1) < grid_count_lambda(entry2);
  });
  const FloatType max_grid_count = grid_count_lambda(*max_it);
  FloatType exp_factor = 20;
  if (viewpoint_entries_.size() >= num_real_viewpoints_ + 100) {
    exp_factor = 1;
  }
  for (auto it = viewpoint_entries_.begin() + num_real_viewpoints_; it != viewpoint_entries_.end(); ++it) {
    const FloatType grid_count = grid_count_lambda(*it);
    AIT_ASSERT(grid_count > 0);
    const FloatType weight = std::exp(-grid_count * exp_factor / max_grid_count);
    weights.push_back(weight);
  }
  viewpoint_sampling_distribution_ = std::discrete_distribution<size_t>(weights.begin(), weights.end());
//  // Display non-empty grid cells
  if (viewpoint_entries_.size() >= num_real_viewpoints_ + 100) {
  //  std::cout << "Viewpoint grid:" << std::endl;
    for (size_t ix = 0; ix < viewpoint_count_grid_.getDimX(); ++ix) {
      for (size_t iy = 0; iy < viewpoint_count_grid_.getDimY(); ++iy) {
        for (size_t iz = 0; iz < viewpoint_count_grid_.getDimZ(); ++iz) {
          const size_t index = viewpoint_count_grid_.getIndex(ix, iy, iz);
          const FloatType grid_count = viewpoint_count_grid_(ix, iy, iz);
          const FloatType probability = std::exp(-grid_count * 20 / max_grid_count);
          grid_cell_probabilities_[index] = probability;
  //        if (grid_count > 0) {
  //          std::cout << "  grid element (" << ix << ", " << iy << ", " << iz << ") has " << grid_count << " viewpoints" << std::endl;
  //        }
        }
      }
    }
  }
  else {
    std::fill(grid_cell_probabilities_.begin(), grid_cell_probabilities_.end(), 1);
  }
}

ViewpointPlanner::ViewpointEntryIndex ViewpointPlanner::addViewpointEntry(ViewpointEntry&& viewpoint_entry) {
  std::unique_lock<std::mutex> lock(mutex_);
  const ViewpointEntryIndex viewpoint_index = addViewpointEntryWithoutLock(std::move(viewpoint_entry));
  lock.unlock();
  return viewpoint_index;
}

const std::pair<std::vector<std::size_t>, std::size_t>& ViewpointPlanner::getConnectedComponents() const {
  if (!viewpoint_graph_components_valid_) {
    std::vector<std::size_t> component(viewpoint_graph_.numVertices());
    std::size_t num_components = boost::connected_components(viewpoint_graph_.boostGraph(), &component.front());
    viewpoint_graph_components_.first = std::move(component);
    viewpoint_graph_components_.second = num_components;
  }
  return viewpoint_graph_components_;
}

bool ViewpointPlanner::generateNextViewpointEntry() {
  const bool verbose = true;

  // If we have failed to sample the second viewpoint too many times we clear the viewpoint graph and start over again.
  const std::size_t max_failed_second_viewpoint_entry_samples = 50;
  if (viewpoint_entries_.size() == num_real_viewpoints_ + 1
      && num_of_failed_viewpoint_entry_samples_ >= max_failed_second_viewpoint_entry_samples) {
    std::cout << "WARNING: Too many failed attempts to sample the second viewpoints. Resetting viewpoint graph." << std::endl;
    reset();
  }

  // Sample viewpoint and add it to the viewpoint graph

  bool found_sample;
  Pose sampled_pose;
  ViewpointEntryIndex reference_viewpoint_index = (ViewpointEntryIndex)-1;

//  if (verbose) {
//    std::cout << "Trying to sample viewpoint" << std::endl;
//  }
  if (viewpoint_entries_.size() <= num_real_viewpoints_ && options_.isSet("drone_start_position")) {
    const Pose drone_start_pose = Pose::createFromImageToWorldTransformation(options_.drone_start_position, Quaternion::Identity());
    std::tie(found_sample, sampled_pose) = sampleSurroundingPose(drone_start_pose);
  }
  else {
    const bool sample_without_reference = random_.sampleBernoulli(options_.viewpoint_sample_without_reference_probability);
    if (sample_without_reference) {
      std::tie(found_sample, sampled_pose) = samplePose(pose_sample_bbox_, drone_extent_);
    }
    else {
      const auto viewpoint_it = sampleViewpointByGridCounts(
          viewpoint_entries_.cbegin() + num_real_viewpoints_, viewpoint_entries_.cend());
      if (viewpoint_it == viewpoint_entries_.cend()) {
        found_sample = false;
      }
      else {
        const Pose& reference_pose = viewpoint_it->viewpoint.pose();
        std::tie(found_sample, sampled_pose) = sampleSurroundingPose(reference_pose);
        const size_t grid_index = viewpoint_count_grid_.getIndex(viewpoint_count_grid_.getGridIndices(reference_pose.getWorldPosition()));
        const FloatType prob = grid_cell_probabilities_[grid_index];
        const FloatType u = random_.sampleUniform();
        if (u > prob) {
          found_sample = false;
        }
      }
    }
  }
  if (!found_sample) {
//    if (verbose) {
//      std::cout << "Failed to sample pose." << i << std::endl;
//    }
    ++num_of_failed_viewpoint_entry_samples_;
    return false;
  }

  const Viewpoint viewpoint = getVirtualViewpoint(sampled_pose);

  // Check distance to other viewpoints and discard if too close
  const std::size_t dist_knn = options_.viewpoint_discard_dist_knn;
  const FloatType dist_thres_square = options_.viewpoint_discard_dist_thres_square;
  const std::size_t dist_count_thres = options_.viewpoint_discard_dist_count_thres;
  const FloatType dist_real_thres_square = options_.viewpoint_discard_dist_real_thres_square;
  static std::vector<ViewpointANN::IndexType> knn_indices;
  static std::vector<ViewpointANN::DistanceType> knn_distances;
  knn_indices.resize(dist_knn);
  knn_distances.resize(dist_knn);
  viewpoint_ann_.knnSearch(sampled_pose.getWorldPosition(), dist_knn, &knn_indices, &knn_distances);
  std::size_t too_close_count = 0;
//  for (ViewpointANN::IndexType viewpoint_index : knn_indices) {
//    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
//    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
  for (std::size_t i = 0; i < knn_distances.size(); ++i) {
    const ViewpointANN::DistanceType dist_square = knn_distances[i];
    const ViewpointEntryIndex viewpoint_index = knn_indices[i];
    if (viewpoint_index < num_real_viewpoints_ && dist_square < dist_real_thres_square) {
//      if (verbose) {
//        std::cout << "Rejected: Too close to real viewpoint" << std::endl;
//      }
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }
//    std::cout << "dist_square=" << dist_square << std::endl;
    if (dist_square < dist_thres_square) {
      ++too_close_count;
//      if (verbose) {
//        std::cout << "dist_square=" << dist_square << std::endl;
//      }
      if (too_close_count > dist_count_thres) {
        break;
      }
    }
  }

//  // Test code for approximate nearest neighbor search
//  static std::vector<ViewpointANN::IndexType> knn_indices2;
//  static std::vector<ViewpointANN::DistanceType> knn_distances2;
//  knn_indices2.resize(dist_knn);
//  knn_distances2.resize(dist_knn);
//  viewpoint_ann_.knnSearchExact(pose.getWorldPosition(), dist_knn, &knn_indices2, &knn_distances2);
//  std::size_t too_close_count2 = 0;
//  for (ViewpointANN::IndexType viewpoint_index : knn_indices2) {
//    const ViewpointEntry& other_viewpoint = viewpoint_entries_[viewpoint_index];
//    FloatType dist_square = (pose.getWorldPosition() - other_viewpoint.viewpoint.pose().getWorldPosition()).squaredNorm();
////    std::cout << "dist_square=" << dist_square << std::endl;
//    if (dist_square < dist_thres_square) {
//      ++too_close_count2;
////      if (verbose) {
////        std::cout << "dist_square=" << dist_square << std::endl;
////      }
//      if (too_close_count2 > dist_count_thres) {
//        break;
//      }
//    }
//  }
//  // This is not necessarily true because of approximate nearest neighbor search but for testing we can quickly see if there is a bug.
//  if (too_close_count != too_close_count2) {
//    std::cerr << "TEST ERROR: too_close_count != too_close_count2" << std::endl;
//    std::cerr << "too_close_count=" << too_close_count << ", too_close_count2=" << too_close_count2 << std::endl;
//  }
//  if (too_close_count > too_close_count2) {
//    std::cout << "too_close_count > too_close_count2" << std::endl;
//    for (std::size_t i = 0; i < knn_indices.size(); ++i) {
//      std::cout << "knn_indices[" << i << "]=" << knn_indices[i] << std::endl;
//      std::cout << "knn_distances[" << i << "]=" << knn_distances[i] << std::endl;
//      std::cout << "getPoint(" << i << ")=" << viewpoint_ann_.getPoint(i) << std::endl;
//      std::cout << "viewpoint_entry[" << knn_indices[i] << "]=" << viewpoint_entries_[knn_indices[i]].viewpoint.pose().getWorldPosition().transpose() << std::endl;
//      std::cout << "knn_indices2[" << i << "]=" << knn_indices2[i] << std::endl;
//      std::cout << "knn_distances2[" << i << "]=" << knn_distances2[i] << std::endl;
//      std::cout << "viewpoint_entry[" << knn_indices2[i] << "]=" << viewpoint_entries_[knn_indices2[i]].viewpoint.pose().getWorldPosition().transpose() << std::endl;
//    }
//  }
//  AIT_ASSERT(too_close_count <= too_close_count2);
//  // End of testing code
//  if (verbose) {
//    std::cout << "too_close_count=" << too_close_count << std::endl;
//  }

  if (too_close_count > dist_count_thres) {
//    if (verbose) {
//      std::cout << "Rejected: Too close to other viewpoints" << std::endl;
//    }
    ++num_of_failed_viewpoint_entry_samples_;
    return false;
  }

  bool found_motion = false;
  SE3Motion se3_motion;
  if (reference_viewpoint_index == (ViewpointEntryIndex)-1 && viewpoint_entries_.size() > num_real_viewpoints_) {
    // Make sure we can find at least one neighbour that is reachable and matchable. Otherwise, discard.
    const std::size_t knn = options_.viewpoint_sample_motion_and_matching_knn;
    static std::vector<ViewpointANN::IndexType> knn_indices;
    static std::vector<ViewpointANN::DistanceType> knn_distances;
    knn_indices.resize(knn);
    knn_distances.resize(knn);
    viewpoint_ann_.knnSearch(sampled_pose.getWorldPosition(), knn, &knn_indices, &knn_distances);
    for (std::size_t i = 0; i < knn_distances.size(); ++i) {
      const ViewpointANN::DistanceType dist_square = knn_distances[i];
      const ViewpointEntryIndex other_viewpoint_index = knn_indices[i];
      if (other_viewpoint_index < num_real_viewpoints_) {
        continue;
      }
      const Viewpoint& other_viewpoint = viewpoint_entries_[other_viewpoint_index].viewpoint;
      const std::unordered_set<Point3DId>& visible_points = computeVisibleSparsePoints(viewpoint);
      const std::unordered_set<Point3DId>& other_visible_points = getCachedVisibleSparsePoints(other_viewpoint_index);
      const bool matchable = isSparseMatchable(viewpoint, other_viewpoint, visible_points, other_visible_points);
      if (!matchable) {
        continue;
      }
      const Pose& other_pose = other_viewpoint.pose();
      std::tie(se3_motion, found_motion) = motion_planner_.findMotion(other_pose, sampled_pose);
      if (found_motion) {
        reference_viewpoint_index = other_viewpoint_index;
        break;
      }
    }
    if (!found_motion) {
      std::cout << "Unable to find motion to new sampled pose" << std::endl;
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }
    FloatType motion_distance = 0;
    if (se3_motion.poses.size() >= 2) {
      for (auto pose_it = se3_motion.poses.begin() + 1; pose_it != se3_motion.poses.end(); ++pose_it) {
        motion_distance += (pose_it->getWorldPosition() - (pose_it - 1)->getWorldPosition()).norm();
      }
    }
#if !AIT_RELEASE
    AIT_ASSERT(ait::isApproxEqual(se3_motion.distance, motion_distance, FloatType(1e-2)));
#endif
  }
  else if (reference_viewpoint_index != (ViewpointEntryIndex)-1
      && reference_viewpoint_index >= num_real_viewpoints_) {
    // Make sure that reference viewpoint is reachable and matchable. Otherwise, discard.
    const Viewpoint& other_viewpoint = viewpoint_entries_[reference_viewpoint_index].viewpoint;
    const bool matchable = isSparseMatchable(viewpoint, other_viewpoint);
    if (!matchable) {
      std::cout << "Unable to match sample reference from new sampled viewpoint" << std::endl;
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }
    const Pose& other_pose = other_viewpoint.pose();
    std::tie(se3_motion, found_motion) = motion_planner_.findMotion(other_pose, sampled_pose);
    if (!found_motion) {
      std::cout << "Unable to find motion from sample reference to new sampled pose" << std::endl;
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }
  }

  // Compute observed voxels and information and discard if too few voxels or too little information
  try {
    const bool ignore_voxels_with_zero_information = true;
    std::pair<VoxelWithInformationSet, FloatType> raycast_result =
        getRaycastHitVoxelsWithInformationScore(viewpoint, ignore_voxels_with_zero_information);
    VoxelWithInformationSet& voxel_set = raycast_result.first;
    FloatType total_information = raycast_result.second;
  //    if (verbose) {
  //      std::cout << "voxel_set.size()=" << voxel_set.size() << std::endl;
  //    }
    if (voxel_set.size() < options_.viewpoint_min_voxel_count) {
      if (verbose) {
        std::cout << "voxel_set.size() < options_.viewpoint_min_voxel_count" << std::endl;
      }
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }
  //  if (verbose) {
  //    std::cout << "total_information=" << total_information << std::endl;
  //  }
    if (total_information < options_.viewpoint_min_information) {
      if (verbose) {
        std::cout << "total_information < options_.viewpoint_min_information" << std::endl;
      }
      ++num_of_failed_viewpoint_entry_samples_;
      return false;
    }

  //  if (verbose) {
  //    std::cout << "pose=" << pose << ", voxel_set=" << voxel_set.size() << ", total_information=" << total_information << std::endl;
  //  }

    const ViewpointEntryIndex new_viewpoint_index = addViewpointEntry(
        ViewpointEntry(Viewpoint(&virtual_camera_, sampled_pose), total_information, std::move(voxel_set)));

    if (found_motion) {
      ViewpointMotion motion({ new_viewpoint_index, reference_viewpoint_index }, { se3_motion });
      addViewpointMotion(std::move(motion));
    }

    num_of_failed_viewpoint_entry_samples_ = 0;
    return true;
  }

  // TODO: Should be a exception for raycast
  catch (const ait::Error& err) {
    std::cout << "Raycast failed: " << err.what() << std::endl;
    return false;
  }
}
