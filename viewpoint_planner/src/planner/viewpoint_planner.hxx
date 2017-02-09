//==================================================
// viewpoint_planner.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 5, 2017
//==================================================

template <typename IteratorT>
std::tuple<bool, ViewpointPlanner::Pose, std::size_t>
ViewpointPlanner::sampleSurroundingPose(IteratorT first, IteratorT last) const {
  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
  IteratorT it = first + index;
//  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
  const Pose& pose = it->viewpoint.pose();
  bool found_pose;
  Pose sampled_pose;
  std::tie(found_pose, sampled_pose) = sampleSurroundingPose(pose);
  return std::make_tuple(found_pose, sampled_pose, index);
}

//template <typename IteratorT>
//std::pair<bool, ViewpointPlanner::Pose> ViewpointPlanner::sampleSurroundingPoseFromEntries(IteratorT first, IteratorT last) const {
//  AIT_ASSERT_STR(last - first > 0, "Unable to sample surrounding pose from empty pose set");
//  std::size_t index = random_.sampleUniformIntExclusive(0, last - first);
//  IteratorT it = first + index;
////  std::cout << "index=" << index << ", last-first=" << (last-first) << std::endl;
//  const ViewpointEntryIndex viewpoint_index = *it;
//  const Pose& pose = viewpoint_entries_[viewpoint_index].viewpoint.pose();
//  return sampleSurroundingPose(pose);
//}

template <typename Iterator>
std::pair<bool, ViewpointPlanner::ViewpointEntryIndex> ViewpointPlanner::canVoxelBeTriangulated(
    const ViewpointPath& viewpoint_path, const ViewpointPathComputationData& comp_data,
    const ViewpointEntry& new_viewpoint, Iterator it) const {
  if (it != comp_data.voxel_observation_counts.cend()) {
    const VoxelType* voxel = it->first;
    const Vector3 voxel_center = voxel->getBoundingBox().getCenter();
    const Vector3 view_direction_1 = (new_viewpoint.viewpoint.pose().getWorldPosition() - voxel_center).normalized();
    const std::vector<ViewpointEntryIndex>& observing_entries = it->second.observing_entries;
    // Check if voxel can be triangulated
    for (auto other_it = observing_entries.cbegin(); other_it != observing_entries.cend(); ++other_it) {
      const ViewpointEntry& other_viewpoint_entry = viewpoint_entries_[*other_it];
      const Vector3 view_direction_2 = (other_viewpoint_entry.viewpoint.pose().getWorldPosition() - voxel_center).normalized();
      const FloatType cos_angle = view_direction_1.dot(view_direction_2);
      if (cos_angle <= triangulation_max_cos_angle_) {
        return std::make_pair(true, *other_it);
      }
    }
  }
  return std::make_pair(false, (ViewpointEntryIndex)-1);
}
